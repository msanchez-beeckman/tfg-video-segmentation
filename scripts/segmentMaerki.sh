#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
TRACKLOCATION="${TFGLOCATION}/test/tracks"
WEIGHTLOCATION="${TFGLOCATION}/results/weights"

usage () { echo "Usage: $0 [-b] [-r radiusTextureless] [-g bgBiasTextureless] [-u lambdaU] [-s lambdaS] [-e minEdgeCost] [-t threshold] datasetName frameLimit"; }

BROXFLAG=""
TRACKSUFFIX=".txt"
rFLAG=""
gFLAG=""
uFLAG=""
sFLAG=""
eFLAG=""
tFLAG=""

while getopts :br:g:u:s:e:t: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        r) rFLAG="--tradius=$OPTARG";;
        g) gFLAG="--tbgbias=$OPTARG";;
        u) uFLAG="--lambdau=$OPTARG";;
        s) sFLAG="--lambdas=$OPTARG";;
        e) eFLAG="--minedgecost=$OPTARG";;
        t) tFLAG="--threshold=$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

mkdir -p ${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}
rm ${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}/*
${TFGLOCATION}/bin/bilateralVidSeg --outfolder=${TFGLOCATION}/results/bvsegmentation/${DATASETNAME}/ ${BROXFLAG} ${rFLAG} ${gFLAG} ${uFLAG} ${sFLAG} ${eFLAG} ${tFLAG} ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}${TRACKSUFFIX} ${WEIGHTLOCATION}/${DATASETNAME}.txt
