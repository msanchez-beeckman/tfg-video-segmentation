
#include <cmath>
#include <opencv4/opencv2/imgproc.hpp>
#include "Tracking.h"

namespace tfg {

    void computeStructureTensorEigenvalues(const cv::Mat &image, cv::Mat &lambda1, cv::Mat &lambda2, double rho) {
        lambda1.release();
        lambda2.release();
        cv::Mat floatImage;
        image.convertTo(floatImage, CV_32F);
        cv::cvtColor(floatImage, floatImage, cv::COLOR_BGR2GRAY);

        // Compute first derivatives of the image
        cv::Mat Ix, Iy;
        cv::Sobel(floatImage, Ix, CV_32F, 1, 0, 3);
        cv::Sobel(floatImage, Iy, CV_32F, 0, 1, 3);

        // Get squares of the first derivatives
        cv::Mat Ix2, Iy2, IxIy;
        cv::multiply(Ix, Ix, Ix2);
        cv::multiply(Iy, Iy, Iy2);
        cv::multiply(Ix, Iy, IxIy);

        // Get components of the structure tensor
        cv::Mat J11, J22, J12;
        cv::GaussianBlur(Ix2, J11, cv::Size(0, 0), rho, rho);
        cv::GaussianBlur(Iy2, J22, cv::Size(0, 0), rho, rho);
        cv::GaussianBlur(IxIy, J12, cv::Size(0, 0), rho, rho);

        // Get the matrices containing every eigenvalue
        cv::Mat tmp1, tmp2, tmp3, tmp4;
        tmp1 = J11 + J22;
        tmp2 = J11 - J22;
        cv::multiply(tmp2, tmp2, tmp2);
        cv::multiply(J12, J12, tmp3);
        cv::sqrt(tmp2 + 4.0 * tmp3, tmp4);

        lambda1 = tmp1 + tmp4;
        lambda2 = tmp1 - tmp4;
    }

    void addTracksToUncoveredZones(const cv::Mat &image, int frame, tfg::TrackTable &trackTable, std::vector<float> &weights, int trackDensity, int coverRadius, double rho) {
        cv::Mat uncovered(image.size(), CV_8UC1, cv::Scalar(255));
        for(unsigned int i = 0; i < trackTable.numberOfTracks(); i++) {
            const int lastFrameOfTrack = trackTable.firstFrameOfTrack(i) + trackTable.durationOfTrack(i) - 1;
            if(lastFrameOfTrack != frame) continue;

            const std::vector<cv::Vec2f> pointsInTrack = trackTable.pointsInTrack(i);
            const cv::Vec2f pointInFrame = pointsInTrack[frame - trackTable.firstFrameOfTrack(i)];

            const int COL = std::round(pointInFrame(0));
            const int ROW = std::round(pointInFrame(1));
            cv::circle(uncovered, cv::Point2i(COL, ROW), coverRadius, cv::Scalar(0), -1);
        }

        cv::Mat lambda1, lambda2;
        tfg::computeStructureTensorEigenvalues(image, lambda1, lambda2, rho);
        const float meanSecondEigenvalue = cv::mean(lambda2)(0);

        for(int r = trackDensity; r < image.rows - trackDensity; r += trackDensity) {
            const float* rowPtrLambda = lambda2.ptr<float>(r);
            const unsigned char* rowPtrUncovered = uncovered.ptr<unsigned char>(r);
            for(int c = trackDensity; c < image.cols - trackDensity; c += trackDensity) {
                const bool noExtraTrackNeeded = rowPtrUncovered[c] == 0;
                if(noExtraTrackNeeded) continue;

                const int distanceToClosestBoundary = std::min(std::min(c, image.cols - c), std::min(r, image.rows - r));
                // const int distanceToClosestBoundary = std::min(std::min(std::min(c, r), image.cols - c), image.rows - r);

                const float relativeDistToBoundary = std::exp(-0.1f * distanceToClosestBoundary);
                const bool tooSmallEigenvalue = rowPtrLambda[c] < meanSecondEigenvalue * (0.1f + relativeDistToBoundary);
                if(tooSmallEigenvalue) continue;

                cv::Vec2f vec;
                vec(0) = static_cast<float>(c);
                vec(1) = static_cast<float>(r);
                const std::vector<cv::Vec2f> coordinates = {vec};
                tfg::Track track(coordinates, frame);
                trackTable.addTrack(track);
                weights.push_back(1.0f);
            }
        }
    }

    void followExistingTracks(const cv::Mat &flow, const cv::Mat &rflow, int frame, tfg::TrackTable &trackTable) {
        cv::Mat fwdFlowDerivX, fwdFlowDerivY;
        cv::Sobel(flow, fwdFlowDerivX, CV_32F, 1, 0, 3);
        cv::Sobel(flow, fwdFlowDerivY, CV_32F, 0, 1, 3);
        for(unsigned int i = 0; i < trackTable.numberOfTracks(); i++) {
            const int lastFrameOfTrack = trackTable.firstFrameOfTrack(i) + trackTable.durationOfTrack(i) - 1;
            if(lastFrameOfTrack != frame) continue;

            const std::vector<cv::Vec2f> pointsInTrack = trackTable.pointsInTrack(i);
            const cv::Vec2f pointInFrame = pointsInTrack[frame - trackTable.firstFrameOfTrack(i)];

            const int pointColFloor = std::floor(pointInFrame(0));
            const int pointRowFloor = std::floor(pointInFrame(1));
            const int pointColCeil = pointColFloor + 1;
            const int pointRowCeil = pointRowFloor + 1;
            const float originAlphaX = pointInFrame(0) - pointColFloor;
            const float originAlphaY = pointInFrame(1) - pointRowFloor;

            const cv::Vec2f interpolatedFlow = (1-originAlphaY)*((1-originAlphaX)*(flow.at<cv::Vec2f>(pointRowFloor, pointColFloor)) + originAlphaX*(flow.at<cv::Vec2f>(pointRowFloor, pointColCeil)))
                                                + originAlphaY*((1-originAlphaX)*(flow.at<cv::Vec2f>(pointRowCeil, pointColFloor)) + originAlphaX*(flow.at<cv::Vec2f>(pointRowCeil, pointColCeil)));
            const cv::Vec2f destinationPoint = pointInFrame + interpolatedFlow;
            const int destColFloor = std::floor(destinationPoint(0));
            const int destRowFloor = std::floor(destinationPoint(1));
            const int destColCeil = destColFloor + 1;
            const int destRowCeil = destRowFloor + 1;
            const float destAlphaX = destinationPoint(0) - destColFloor;
            const float destAlphaY = destinationPoint(1) - destRowFloor;
            if(destColFloor < 0 || destColCeil >= rflow.cols || destRowFloor < 0 || destRowCeil >= rflow.rows) continue;

            const cv::Vec2f interpolatedBwdFlow = (1-destAlphaY)*((1-destAlphaX)*(rflow.at<cv::Vec2f>(destRowFloor, destColFloor)) + destAlphaX*(rflow.at<cv::Vec2f>(destRowFloor, destColCeil)))
                                                   + destAlphaY*((1-destAlphaX)*(rflow.at<cv::Vec2f>(destRowCeil, destColFloor)) + destAlphaX*(rflow.at<cv::Vec2f>(destRowCeil, destColCeil)));
            const cv::Vec2f returnPoint = destinationPoint + interpolatedBwdFlow;

            const float leftRightError = cv::norm(pointInFrame - returnPoint, cv::NORM_L2SQR);
            const float fwdFlowNorm = cv::norm(interpolatedFlow, cv::NORM_L2SQR);
            const float bwdFlowNorm = cv::norm(interpolatedBwdFlow, cv::NORM_L2SQR);
            if(leftRightError > 0.01f * (fwdFlowNorm + bwdFlowNorm) + 0.5f) continue;

            const cv::Vec2f flowXDerivs = (1-originAlphaY)*((1-originAlphaX)*(fwdFlowDerivX.at<cv::Vec2f>(pointRowFloor, pointColFloor)) + originAlphaX*(fwdFlowDerivX.at<cv::Vec2f>(pointRowFloor, pointColCeil)))
                                           + originAlphaY*((1-originAlphaX)*(fwdFlowDerivX.at<cv::Vec2f>(pointRowCeil, pointColFloor)) + originAlphaX*(fwdFlowDerivX.at<cv::Vec2f>(pointRowCeil, pointColCeil)));
            const cv::Vec2f flowYDerivs = (1-originAlphaY)*((1-originAlphaX)*(fwdFlowDerivY.at<cv::Vec2f>(pointRowFloor, pointColFloor)) + originAlphaX*(fwdFlowDerivY.at<cv::Vec2f>(pointRowFloor, pointColCeil)))
                                           + originAlphaY*((1-originAlphaX)*(fwdFlowDerivY.at<cv::Vec2f>(pointRowCeil, pointColFloor)) + originAlphaX*(fwdFlowDerivY.at<cv::Vec2f>(pointRowCeil, pointColCeil)));
            const float sqrFlowGradientSum = cv::norm(flowXDerivs, cv::NORM_L2SQR) + cv::norm(flowYDerivs, cv::NORM_L2SQR);
            // if(sqrFlowGradientSum > 0.01f * fwdFlowNorm + 0.002f) continue;

            trackTable.addPointToTrack(destinationPoint, i);
        }
    }
}