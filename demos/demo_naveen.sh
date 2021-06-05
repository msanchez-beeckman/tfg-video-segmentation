#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DEMORESULTSFOLDER=${TFGLOCATION}/demo-results/naveen

mkdir -p ${DEMORESULTSFOLDER}
bash ${TFGLOCATION}/scripts/compute_shrunk_flows.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31 1
bash ${TFGLOCATION}/scripts/dense_tracking.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31
bash ${TFGLOCATION}/scripts/track_segment_naveen.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31
bash ${TFGLOCATION}/scripts/densify_maerki.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31