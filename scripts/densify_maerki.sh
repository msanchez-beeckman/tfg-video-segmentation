#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
RESULTSFOLDER="${TFGLOCATION}/results"

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

while getopts :bd:e:g:Ms:u:r:t:o: opt; do
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
        o) RESULTSFOLDER="$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

TRACKLOCATION="${RESULTSFOLDER}/tracks"
WEIGHTLOCATION="${RESULTSFOLDER}/weights"

DATASETNAME=$1
FRAMELIMIT=$2

mkdir -p ${RESULTSFOLDER}/bvsegmentation/${DATASETNAME}
rm -f ${RESULTSFOLDER}/bvsegmentation/${DATASETNAME}/*
${TFGLOCATION}/bin/bvs ${BROXFLAG} ${dFLAG} ${eFLAG} ${gFLAG} ${MFLAG} ${sFLAG} ${uFLAG} ${rFLAG} ${tFLAG} --outfolder=${RESULTSFOLDER}/bvsegmentation/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${TRACKLOCATION}/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX} ${WEIGHTLOCATION}/${DATASETNAME}.txt
