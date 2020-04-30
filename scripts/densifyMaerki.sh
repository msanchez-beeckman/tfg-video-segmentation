#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
TRACKLOCATION="${TFGLOCATION}/test/tracks"
WEIGHTLOCATION="${TFGLOCATION}/results/weights"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-e minEdgeCost] [-g bgBiasTextureless] [-M] [-s lambdaS] [-u lambdaU] [-r radiusTextureless] [-t threshold] datasetName frameLimit"; }

TRACKSUFFIX=".txt"
BROXFLAG=""
dFLAG=""
eFLAG=""
gFLAG=""
MFLAG=""
sFLAG=""
uFLAG=""
rFLAG=""
tFLAG=""

while getopts :bd:e:g:Ms:u:r:t: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        d) dFLAG="--minTrackDuration=${OPTARG}";;
        e) eFLAG="--minEdgeCost=${OPTARG}";;
        g) gFLAG="--tbgbias=${OPTARG}";;
        M) MFLAG="--multilabel";;
        s) sFLAG="--lambdas=${OPTARG}";;
        u) uFLAG="--lambdau=${OPTARG}";;
        r) rFLAG="--tradius=${OPTARG}";;
        t) tFLAG="--threshold=${OPTARG}";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

mkdir -p ${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}
rm -f ${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}/*
${TFGLOCATION}/bin/bilateralVidSeg ${BROXFLAG} ${dFLAG} ${eFLAG} ${gFLAG} ${MFLAG} ${sFLAG} ${uFLAG} ${rFLAG} ${tFLAG} --outfolder=${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX} ${WEIGHTLOCATION}/${DATASETNAME}.txt
