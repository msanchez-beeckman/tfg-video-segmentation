#!/usr/bin/env bash

usage () { echo "Usage: $0 datasetName frameLimit surroundFrames"; }

[ $# -lt 3 ] && { usage; exit 1; }

DATASETNAME=$1
FRAMELIMIT=$2
SURROUNDFRAMES=$3

# The following variables should be adjusted to the absolute path of the project's directory,
# and to the directory containing the data, respectively.
TFGLOCATION="/home/marco/CLionProjects/tfg_video_segmentation"
DATALOCATION="${TFGLOCATION}/data"

FLOWFOLDER="${TFGLOCATION}/results/flows/${DATASETNAME}"
mkdir -p ${FLOWFOLDER}
echo "${FRAMELIMIT}" > ${FLOWFOLDER}/flows.txt

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    convert ${DATALOCATION}/${DATASETNAME}/${IMAGE}.jpg -resize 160x90\> ${FLOWFOLDER}/shrink_${IMAGE}.jpg
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    CURRENTIMAGE=$(printf "%05d" $i)
    echo "Computing flows using ${CURRENTIMAGE} as source"
    min=$(( (i - SURROUNDFRAMES) < 0 ? 0 : (i - SURROUNDFRAMES) ))
    max=$(( (i + SURROUNDFRAMES) > (FRAMELIMIT - 1) ? (FRAMELIMIT - 1) : (i + SURROUNDFRAMES) ))
    totalflows=$(( max - min ))
    echo "${totalflows}" >> ${FLOWFOLDER}/flows.txt
    for (( j=min; j<=max; j++ )); do
        if [ $j -eq $i ]; then continue; fi
        TARGETIMAGE=$(printf "%05d" $j)
        echo "          using ${TARGETIMAGE} as destination"
        src_flow_tv_l1 -l 0.1 ${FLOWFOLDER}/shrink_${CURRENTIMAGE}.jpg ${FLOWFOLDER}/shrink_${TARGETIMAGE}.jpg ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff ${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff
        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}u.tiff" >> ${FLOWFOLDER}/flows.txt
        echo "${FLOWFOLDER}/${CURRENTIMAGE}to${TARGETIMAGE}v.tiff" >> ${FLOWFOLDER}/flows.txt
    done
done

for (( i=0; i<${FRAMELIMIT}; i++ )); do
    IMAGE=$(printf "%05d" $i)
    rm ${FLOWFOLDER}/shrink_${IMAGE}.jpg
done
