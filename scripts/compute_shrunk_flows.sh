#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DATALOCATION="${TFGLOCATION}/data"
RESULTSFOLDER="${TFGLOCATION}/results"

usage () { echo "Usage: $0 [-b] [-r WIDTHxHEIGHT] datasetName frameLimit surroundFrames"; }

NEWSIZE="854x480"
SPEED=0

while getopts :r:s:o: opt; do
    case $opt in
        r) NEWSIZE="$OPTARG";;
        s) SPEED="$OPTARG";;
        o) RESULTSFOLDER="$OPTARG";;
        :) echo "Missing argument for option -$OPTARG"; exit 1;;
       \?) echo "Unknown option -$OPTARG"; exit 1;;
    esac
done

shift $(( OPTIND - 1 ))

[ $# -lt 3 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2
SURROUNDFRAMES=$3

FLOWFOLDER="${RESULTSFOLDER}/flows/${DATASETNAME}"
mkdir -p ${FLOWFOLDER}
echo "${FRAMELIMIT}" > ${FLOWFOLDER}/surroundingFlows.txt

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    convert ${DATALOCATION}/${DATASETNAME}/${IMAGE}.jpg -resize ${NEWSIZE}\> ${FLOWFOLDER}/shrink_${IMAGE}.jpg
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    CURRENTIMAGE=$(printf "%05d" $i)
    echo "Computing flows using ${CURRENTIMAGE} as source"
    min=$(( (i - SURROUNDFRAMES) < 0 ? 0 : (i - SURROUNDFRAMES) ))
    max=$(( (i + SURROUNDFRAMES) > (FRAMELIMIT - 1) ? (FRAMELIMIT - 1) : (i + SURROUNDFRAMES) ))
    totalflows=$(( max - min ))
    echo "${totalflows}" >> ${FLOWFOLDER}/surroundingFlows.txt
    for (( j=min; j<=max; j++ )); do
        if [ $j -eq $i ]; then continue; fi
        TARGETIMAGE=$(printf "%05d" $j)
        echo "          using ${TARGETIMAGE} as destination"

        ${TFGLOCATION}/bin/optical_flow_dis --speed=${SPEED} "${FLOWFOLDER}/shrink_${CURRENTIMAGE}.jpg" "${FLOWFOLDER}/shrink_${TARGETIMAGE}.jpg" "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff" "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff"

        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff" >> ${FLOWFOLDER}/surroundingFlows.txt
        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff" >> ${FLOWFOLDER}/surroundingFlows.txt
    done
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    rm ${FLOWFOLDER}/shrink_${IMAGE}.jpg
done
