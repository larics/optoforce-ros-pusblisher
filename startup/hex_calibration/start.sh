#!/bin/bash
# HEX calibration session (tmuxinator, LARICS style):   ./start.sh [setup file]

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm -f .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln session.yml .tmuxinator.yml

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=hex_calibration.sh

# start tmuxinator
tmuxinator hex_calibration setup_name=$SETUP_NAME
