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

python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
mkdir -p ${TFGLOCATION}/results/model/${DATASETNAME}
mkdir -p ${TFGLOCATION}/results/weights/
rm ${TFGLOCATION}/results/model/${DATASETNAME}/*
${TFGLOCATION}/bin/motionModel ${BROXFLAG} ${TFGLOCATION}/test/tracks/${DATASETNAME}${TRACKSUFFIX} ${DATALOCATION}/${DATASETNAME}/images.txt -w ${TFGLOCATION}/results/weights/${DATASETNAME}.txt -o ${TFGLOCATION}/results/model/${DATASETNAME}/
