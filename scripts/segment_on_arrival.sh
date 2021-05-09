#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
FLOWLOCATION="${TFGLOCATION}/results/flows"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-e ransacEpsilon] [-i ransacIterations] [-t tau2] datasetName frameLimit"; }

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

PREVTRACKS=""
PREVWEIGHTS=""
for (( i=0; i<$(( FRAMELIMIT - 1 )); i++ )); do
    python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg $i 2 ${DATALOCATION}/${DATASETNAME}/rtImages.txt False
    python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ tiff $i 1 ${FLOWLOCATION}/${DATASETNAME}/rtFlows.txt False
    python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ tiff $((i + 1)) 1 ${FLOWLOCATION}/${DATASETNAME}/rtFlowsr.txt True

    ${TFGLOCATION}/bin/continueTracks --firstNameIndex=$i --outfolder=${TFGLOCATION}/results/realtime/tracks/${DATASETNAME}/ --outweights=${TFGLOCATION}/results/realtime/weights/${DATASETNAME}.txt --trackFilePath=${TFGLOCATION}/results/realtime/tracks/${DATASETNAME}/tracks.txt ${DATALOCATION}/${DATASETNAME}/rtImages.txt ${FLOWLOCATION}/${DATASETNAME}/rtFlows.txt ${FLOWLOCATION}/${DATASETNAME}/rtFlowsr.txt ${PREVTRACKS} ${PREVWEIGHTS}
    ${TFGLOCATION}/bin/motionModel --firstNameIndex=$i --minTrackDuration=2 --outfolder=${TFGLOCATION}/results/realtime/model/${DATASETNAME}/ --outweights=${TFGLOCATION}/results/realtime/weights/${DATASETNAME}.txt ${DATALOCATION}/${DATASETNAME}/rtImages.txt ${TFGLOCATION}/results/realtime/tracks/${DATASETNAME}/tracks.txt ${TFGLOCATION}/results/realtime/weights/${DATASETNAME}.txt
    ${TFGLOCATION}/bin/bilateralVidSeg --firstNameIndex=$i --minTrackDuration=2 --outfolder=${TFGLOCATION}/results/realtime/segmentation/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/rtImages.txt ${TFGLOCATION}/results/realtime/tracks/${DATASETNAME}/tracks.txt ${TFGLOCATION}/results/realtime/weights/${DATASETNAME}.txt
    PREVTRACKS="${TFGLOCATION}/results/realtime/tracks/${DATASETNAME}/tracks.txt"
    PREVWEIGHTS="${TFGLOCATION}/results/realtime/weights/${DATASETNAME}.txt"
done
