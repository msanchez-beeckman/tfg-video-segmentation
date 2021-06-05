#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
RESULTSFOLDER="${TFGLOCATION}/results"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-D] [-l lambda] datasetName frameLimit"; }

TRACKSUFFIX=".txt"
DAVISSUFFIX=""
BROXFLAG=""
dFLAG=""
DFLAG=""
lFLAG=""

while getopts :bd:Dl:o: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        d) dFLAG="--minTrackDuration=${OPTARG}";;
        D) DFLAG="--davis"; DAVISSUFFIX="Davis";;
        l) lFLAG="--lambda=${OPTARG}";;
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

# This variable should also be modified if the seeds for the images are stored elsewhere
SEEDLOCATION="${DATALOCATION}/${DATASETNAME}/seeds"

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
python ${TFGLOCATION}/scripts/list_images.py ${SEEDLOCATION}/ png 0 ${FRAMELIMIT} ${SEEDLOCATION}/seeds.txt True
mkdir -p ${RESULTSFOLDER}/walkedseeds/${DATASETNAME}
mkdir -p ${RESULTSFOLDER}/weights/
rm -f ${RESULTSFOLDER}/walkedseeds/${DATASETNAME}/*
${TFGLOCATION}/bin/random_walk ${BROXFLAG} ${dFLAG} ${DFLAG} ${lFLAG} --outweights=${RESULTSFOLDER}/weights/${DATASETNAME}.txt --outfolder=${RESULTSFOLDER}/walkedseeds/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${SEEDLOCATION}/seeds${DAVISSUFFIX}.txt ${TRACKLOCATION}/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX}
