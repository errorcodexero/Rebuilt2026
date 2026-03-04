#!/bin/bash

REMOTE_HOST="10.14.25.2"
REMOTE_DIR="/home/lvuser/deploy/tuning"
LOCAL_DIR="src/main/deploy/tuning"

ssh "admin@${REMOTE_HOST}" "rm -rf ${REMOTE_DIR} && mkdir -p ${REMOTE_DIR}"
scp "${LOCAL_DIR}"/*.json "admin@${REMOTE_HOST}:${REMOTE_DIR}/"
echo "Tuning files deployed to ${REMOTE_HOST}:${REMOTE_DIR}"