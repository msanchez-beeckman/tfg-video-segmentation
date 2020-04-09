#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# to the directory containing the data, and to the directory containing the tracks file, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
TRACKLOCATION="${TFGLOCATION}/test/tracks"

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

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${TFGLOCATION}/results/model/${DATASETNAME}
mkdir -p ${TFGLOCATION}/results/weights/
rm -f ${TFGLOCATION}/results/model/${DATASETNAME}/*
${TFGLOCATION}/bin/motionModel ${BROXFLAG} ${dFLAG} ${eFLAG} ${iFLAG} ${tFLAG} --outweights=${TFGLOCATION}/results/weights/${DATASETNAME}.txt --outfolder=${TFGLOCATION}/results/model/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX}
