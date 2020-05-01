#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# to the directory containing the data, and to the directory containing the tracks file, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
TRACKLOCATION="${TFGLOCATION}/results/tracks"

usage () { echo "Usage: $0 [-b] [-d minTrackDuration] [-D] [-l lambda] datasetName frameLimit"; }

TRACKSUFFIX=".txt"
DAVISSUFFIX=""
BROXFLAG=""
dFLAG=""
DFLAG=""
lFLAG=""

while getopts :bd:Dl: opt; do
    case $opt in
        b) BROXFLAG="--brox"; TRACKSUFFIX="Brox.dat";;
        d) dFLAG="--minTrackDuration=${OPTARG}";;
        D) DFLAG="--davis"; DAVISSUFFIX="Davis";;
        l) lFLAG="--lambda=${OPTARG}";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2

# This variable should also be modified if the seeds for the images are stored elsewhere
SEEDLOCATION="${DATALOCATION}/${DATASETNAME}/seeds"

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
python ${TFGLOCATION}/scripts/list_images.py ${SEEDLOCATION}/ png 0 ${FRAMELIMIT} ${SEEDLOCATION}/seeds.txt True
mkdir -p ${TFGLOCATION}/results/walkedseeds/${DATASETNAME}
mkdir -p ${TFGLOCATION}/results/weights/
rm -f ${TFGLOCATION}/results/walkedseeds/${DATASETNAME}/*
${TFGLOCATION}/bin/randomWalker ${BROXFLAG} ${dFLAG} ${DFLAG} ${lFLAG} --outweights=${TFGLOCATION}/results/weights/${DATASETNAME}.txt --outfolder=${TFGLOCATION}/results/walkedseeds/${DATASETNAME}/ ${DATALOCATION}/${DATASETNAME}/images.txt ${SEEDLOCATION}/seeds${DAVISSUFFIX}.txt ${TRACKLOCATION}/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}${TRACKSUFFIX}
