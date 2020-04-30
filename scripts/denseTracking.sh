#!/usr/bin/env bash

# The following variables should be adjusted to the absolute path of the project's directory,
# to the directory containing the data, and to the directory containing the tracks file, respectively.
TFGLOCATION="/home/marco/Projects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"
FLOWLOCATION="${TFGLOCATION}/results/flows"

usage () { echo "Usage: $0 [-c coverRadius] [-d density] [-f] [-r rho] datasetName frameLimit"; }

flowFormat="tiff"
cFLAG=""
dFLAG=""
fFLAG=""
rFLAG=""

while getopts :c:d:fr: opt; do
    case $opt in
        c) cFLAG="--coverRadius=${OPTARG}";;
        d) dFLAG="--density=${OPTARG}";;
        f) fFLAG="--flo"; flowFormat="flo";;
        r) rFLAG="--rho=${OPTARG}";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 2 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2


python ${TFGLOCATION}/scripts/list_images.py ${DATALOCATION}/${DATASETNAME}/ jpg 0 ${FRAMELIMIT} ${DATALOCATION}/${DATASETNAME}/images.txt False
python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ ${flowFormat} 0 ${FRAMELIMIT} ${FLOWLOCATION}/${DATASETNAME}/flows.txt ${FLOWLOCATION}/${DATASETNAME}/flows.txt False
python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ ${flowFormat} 0 ${FRAMELIMIT} ${FLOWLOCATION}/${DATASETNAME}/flows.txt ${FLOWLOCATION}/${DATASETNAME}/flowsr.txt True

mkdir -p ${TFGLOCATION}/results/tracks/${DATASETNAME}
rm -f ${TFGLOCATION}/results/tracks/${DATASETNAME}/*
${TFGLOCATION}/bin/continueTracks ${cFLAG} ${dFLAG} ${fFLAG} ${rFLAG} --outfolder=${TFGLOCATION}/results/tracks/${DATASETNAME}/ --trackFilePath=${TFGLOCATION}/results/tracks/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}.txt ${DATALOCATION}/${DATASETNAME}/images.txt ${FLOWLOCATION}/${DATASETNAME}/flows.txt ${FLOWLOCATION}/${DATASETNAME}/flowsr.txt
