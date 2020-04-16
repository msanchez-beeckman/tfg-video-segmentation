#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"

usage () { echo "Usage: $0 [-b] [-r WIDTHxHEIGHT] datasetName frameLimit surroundFrames"; }

LDOF=0
IMAGEFORMAT="jpg"
NEWSIZE="854x480"

while getopts :br: opt; do
    case $opt in
        b) IMAGEFORMAT="ppm"; LDOF=1;;
        R) NEWSIZE="$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 3 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2
SURROUNDFRAMES=$3

FLOWFOLDER="${TFGLOCATION}/results/flows/${DATASETNAME}"
mkdir -p ${FLOWFOLDER}
echo "${FRAMELIMIT}" > ${FLOWFOLDER}/flows.txt

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    convert ${DATALOCATION}/${DATASETNAME}/${IMAGE}.jpg -resize ${NEWSIZE}\> ${FLOWFOLDER}/shrink_${IMAGE}.${IMAGEFORMAT}
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    CURRENTIMAGE=$(printf "%05d" $i)
    echo "Computing flows using ${CURRENTIMAGE} as source"
    min=$(( (i - SURROUNDFRAMES) < 0 ? 0 : (i - SURROUNDFRAMES) ))
    max=$(( (i + SURROUNDFRAMES) > (FRAMELIMIT - 1) ? (FRAMELIMIT - 1) : (i + SURROUNDFRAMES) ))
    totalflows=$(( max - min ))
    echo "${totalflows}" >> ${FLOWFOLDER}/flows.txt
    for (( j=min; j<=max; j++ )); do
        if [ $j -eq $i ]; then continue; fi
        TARGETIMAGE=$(printf "%05d" $j)
        echo "          using ${TARGETIMAGE} as destination"

        if [ $LDOF -eq 1 ]; then
            ${TFGLOCATION}/external/src_flow_ldof ${FLOWFOLDER}/shrink_${CURRENTIMAGE}.${IMAGEFORMAT} ${FLOWFOLDER}/shrink_${TARGETIMAGE}.${IMAGEFORMAT} ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}.flo
            ${TFGLOCATION}/external/src_flow_read_flo ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}.flo ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff
        else
            src_flow_tv_l1 -l 0.1 ${FLOWFOLDER}/shrink_${CURRENTIMAGE}.${IMAGEFORMAT} ${FLOWFOLDER}/shrink_${TARGETIMAGE}.${IMAGEFORMAT} ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff;
        fi

        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff" >> ${FLOWFOLDER}/flows.txt
        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff" >> ${FLOWFOLDER}/flows.txt
    done
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    rm ${FLOWFOLDER}/shrink_${IMAGE}.${IMAGEFORMAT}
done
