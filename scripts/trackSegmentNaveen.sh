#!/usr/bin/env bash

usage () { echo "Usage: $0 [-b] datasetName frameLimit"; }

BROXFLAG=""
TRACKSUFFIX=".txt"

while getopts :b opt; do
    case $opt in
        b) BROXFLAG="-b"; TRACKSUFFIX="Brox.dat";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2


# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"

# This variable should also be modified if the seeds for the images are stored elsewhere
SEEDLOCATION="${DATALOCATION}/${DATASETNAME}/seeds"

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
python ${TFGLOCATION}/scripts/list_images.py ${SEEDLOCATION}/ png ${FRAMELIMIT} ${SEEDLOCATION}/seeds.txt True
mkdir -p ${TFGLOCATION}/results/walkedseeds/${DATASETNAME}
mkdir -p ${TFGLOCATION}/results/walkerprobs/
rm ${TFGLOCATION}/results/walkedseeds/${DATASETNAME}/*
${TFGLOCATION}/bin/randomWalker ${BROXFLAG} ${TFGLOCATION}/test/tracks/${DATASETNAME}${TRACKSUFFIX} ${SEEDLOCATION}/seeds.txt ${DATALOCATION}/${DATASETNAME}/images.txt -w ${TFGLOCATION}/results/walkerprobs/${DATASETNAME}.txt -o ${TFGLOCATION}/results/walkedseeds/${DATASETNAME}/
