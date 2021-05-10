#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
FLOWLOCATION="${TFGLOCATION}/results/flows"

usage () { echo "Usage: $0 [-B] [-S] [-c coverRadius] [-d density] [-f] [-r rho] datasetName frameLimit"; }

flowFormat="tiff"
BFLAG=""
SFLAG=""
cFLAG=""
dFLAG=""
fFLAG=""
rFLAG=""

while getopts :BSc:d:fr: opt; do
    case $opt in
        B) BFLAG="--motionBoundary";;
        S) SFLAG="--structure";;
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
python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ ${flowFormat} 0 ${FRAMELIMIT} ${FLOWLOCATION}/${DATASETNAME}/flows.txt False
python ${TFGLOCATION}/scripts/list_flows.py ${FLOWLOCATION}/${DATASETNAME}/ ${flowFormat} 0 ${FRAMELIMIT} ${FLOWLOCATION}/${DATASETNAME}/flowsr.txt True

mkdir -p ${TFGLOCATION}/results/tracks/${DATASETNAME}
${TFGLOCATION}/bin/continue_tracks ${BFLAG} ${SFLAG} ${cFLAG} ${dFLAG} ${fFLAG} ${rFLAG} --outfolder=${TFGLOCATION}/results/tracks/${DATASETNAME}/ --trackFilePath=${TFGLOCATION}/results/tracks/${DATASETNAME}/${DATASETNAME}${FRAMELIMIT}.txt ${DATALOCATION}/${DATASETNAME}/images.txt ${FLOWLOCATION}/${DATASETNAME}/flows.txt ${FLOWLOCATION}/${DATASETNAME}/flowsr.txt
