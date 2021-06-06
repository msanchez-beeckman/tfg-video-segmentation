#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
RESULTSFOLDER="${TFGLOCATION}/results"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-e ransacEpsilon] [-i ransacIterations] [-t tau2] datasetName frameLimit"; }

TRACKSUFFIX=".txt"
BROXFLAG=""
dFLAG=""
eFLAG=""
iFLAG=""
tFLAG=""

while getopts :bd:e:i:t:o: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        d) dFLAG="--minTrackDuration=${OPTARG}";;
        e) eFLAG="--ransacEpsilon=${OPTARG}";;
        i) iFLAG="--ransacIterations=${OPTARG}";;
        t) tFLAG="--tau2=${OPTARG}";;
        o) RESULTSFOLDER="$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

TRACKLOCATION="${RESULTSFOLDER}/tracks"
DATASETNAME=$1
FRAMELIMIT=$2

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${RESULTSFOLDER}/model/${DATASETNAME}
mkdir -p ${RESULTSFOLDER}/weights/
rm -f ${RESULTSFOLDER}/model/${DATASETNAME}/*
${TFGLOCATION}/bin/bgmm ${BROXFLAG} ${dFLAG} ${eFLAG} ${iFLAG} ${tFLAG} --outweights=${RESULTSFOLDER}/weights/${DATASETNAME}.txt --outfolder=${RESULTSFOLDER}/model/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX}
