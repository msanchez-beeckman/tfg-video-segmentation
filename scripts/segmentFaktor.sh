#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
FLOWLOCATION="${TFGLOCATION}/results/flows"

usage () { echo "Usage: $0 [-s spSize] [-F windowRadius] [-L NNperFrame] [-S sigma2] [-T iterations] [-t threshold] [-r removeBlobs] datasetName frameLimit"; }

sFLAG=""
FFLAG=""
LFLAG=""
SFLAG=""
TFLAG=""
tFLAG=""
rFLAG=""

while getopts :s:F:L:S:T:t:r opt; do
    case $opt in
        s) sFLAG="-s $OPTARG";;
        F) FFLAG="-F $OPTARG";;
        L) LFLAG="-L $OPTARG";;
        S) SFLAG="-S $OPTARG";;
        T) TFLAG="-T $OPTARG";;
        t) tFLAG="-t $OPTARG";;
        r) rFLAG="-r";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${TFGLOCATION}/results/nlcsegmentation/${DATASETNAME}
rm ${TFGLOCATION}/results/nlcsegmentation/${DATASETNAME}/*
${TFGLOCATION}/bin/consensusVoting ${DATALOCATION}/${DATASETNAME}/images.txt ${FLOWLOCATION}/${DATASETNAME}/flows.txt ${sFLAG} ${FFLAG} ${LFLAG} ${SFLAG} ${TFLAG} ${tFLAG} ${rFLAG} -o ${TFGLOCATION}/results/nlcsegmentation/${DATASETNAME}/
