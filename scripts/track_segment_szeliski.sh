#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
TRACKLOCATION="${TFGLOCATION}/results/tracks"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-e ransacEpsilon] [-i ransacIterations] [-t tau2] datasetName frameLimit"; }

TRACKSUFFIX=".txt"
BROXFLAG=""
dFLAG=""
eFLAG=""
iFLAG=""
tFLAG=""

while getopts :bd:e:i:t: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        d) dFLAG="--minTrackDuration=${OPTARG}";;
        e) eFLAG="--ransacEpsilon=${OPTARG}";;
        i) iFLAG="--ransacIterations=${OPTARG}";;
        t) tFLAG="--tau2=${OPTARG}";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${TFGLOCATION}/results/model/${DATASETNAME}
mkdir -p ${TFGLOCATION}/results/weights/
rm -f ${TFGLOCATION}/results/model/${DATASETNAME}/*
${TFGLOCATION}/bin/bgmm ${BROXFLAG} ${dFLAG} ${eFLAG} ${iFLAG} ${tFLAG} --outweights=${TFGLOCATION}/results/weights/${DATASETNAME}.txt --outfolder=${TFGLOCATION}/results/model/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX}
