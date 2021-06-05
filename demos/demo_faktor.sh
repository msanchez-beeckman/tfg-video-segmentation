#!/usr/bin/env bash

TFGLOCATION="$(dirname $0)/.."
DEMORESULTSFOLDER=${TFGLOCATION}/demo-results/faktor

mkdir -p ${DEMORESULTSFOLDER}
bash ${TFGLOCATION}/scripts/compute_shrunk_flows.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31 3
bash ${TFGLOCATION}/scripts/segment_faktor.sh -o ${DEMORESULTSFOLDER} bmx-bumps 31