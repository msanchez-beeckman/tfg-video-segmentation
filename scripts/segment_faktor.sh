#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
RESULTSFOLDER="${TFGLOCATION}/results"

usage () { echo "Usage: $0 [-d minDomMotion] [-s spSize] [-O] [-F windowRadius] [-L NNperFrame] [-S sigma2] [-T iterations] [-t threshold] [-r removeBlobs] datasetName frameLimit"; }

dFLAG=""
sFLAG=""
OFLAG=""
FFLAG=""
LFLAG=""
SFLAG=""
TFLAG=""
tFLAG=""
rFLAG=""

while getopts :d:s:OF:L:S:T:t:r:o: opt; do
    case $opt in
        d) dFLAG="--minDomMotion=${OPTARG}";;
        s) sFLAG="--spsize=${OPTARG}";;
        O) OFLAG="--slico";;
        F) FFLAG="-F=${OPTARG}";;
        L) LFLAG="-L=${OPTARG}";;
        S) SFLAG="--sigma2=${OPTARG}";;
        T) TFLAG="--iterations=${OPTARG}";;
        t) tFLAG="--threshold=${OPTARG}";;
        r) rFLAG="--removeBlobsTh=${OPTARG}";;
        o) RESULTSFOLDER="$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

FLOWLOCATION="${RESULTSFOLDER}/flows"
DATASETNAME=$1
FRAMELIMIT=$2

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${RESULTSFOLDER}/nlcsegmentation/${DATASETNAME}
rm -f ${RESULTSFOLDER}/nlcsegmentation/${DATASETNAME}/*
${TFGLOCATION}/bin/nlcv ${dFLAG} ${sFLAG} ${OFLAG} ${FFLAG} ${LFLAG} ${SFLAG} ${TFLAG} ${tFLAG} ${rFLAG} --outfolder=${RESULTSFOLDER}/nlcsegmentation/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${FLOWLOCATION}/${DATASETNAME}/surroundingFlows.txt
