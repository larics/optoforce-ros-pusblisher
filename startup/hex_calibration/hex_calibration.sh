#!/bin/bash
# Setup of the HEX calibration session: sourced by tmuxinator in every pane.
#   ./start.sh [setup file]                     start (default: this file)
#   tmux kill-session -t hex_calibration        stop everything
# The reference sensor's topic and rated range, and all calibration settings, are in
# cfg/hex_calibration_params.yaml of this package.

# Drivers to start (empty = you start them yourself)
export REFERENCE_DRIVER="roslaunch ati_gamma_ros ati_gamma.launch rate:=1000"   # any WrenchStamped publisher
export HEX_DRIVER="roslaunch optoforce_sensor onrobot_hex_sensor.launch"

# This package and its settings (everything the calibration reads and writes stays in it)
export HEXCAL_PKG=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
export HEXCAL_PARAMS=$HEXCAL_PKG/cfg/hex_calibration_params.yaml
hexcal_param() { python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))[sys.argv[2]])" "$HEXCAL_PARAMS" "$1"; }
export REFERENCE_TOPIC=$(hexcal_param reference_topic)
export HEX_TOPIC=$(hexcal_param hex_topic)
export HEXCAL_OUTPUT=$(hexcal_param output_topic)

# Helpers: the LARICS images define waitForRos / waitFor; these are fallbacks
type waitForRos >/dev/null 2>&1 || waitForRos() {
  until rostopic list >/dev/null 2>&1; do echo "waiting for roscore ..."; sleep 1; done
}
type waitFor >/dev/null 2>&1 || waitFor() {
  until timeout 3 rostopic echo -n 1 "$1" >/dev/null 2>&1; do echo "waiting for $1 ..."; sleep 1; done
}
# start a driver unless it is empty
hexcal_driver() { if [ -n "$1" ]; then eval "$1"; else echo "start this driver yourself"; fi; }

# Cheat sheet (calibration window, right pane); arrow-up recalls the commands
hexcal_help() {
  cat <<HELP
 HEX calibration against a reference F/T sensor      tmux: Ctrl+b then 0-3 = window, arrows = pane
 windows: 0 roscore + drivers   1 calibration (dashboard left)   2 plots   3 verify (after the fit)
 reference: $REFERENCE_TOPIC      HEX: $HEX_TOPIC

 dashboard keys:  [s] start/stop session   [f] fit   [z] tare   [d] delete last   [m] model   [q] quit
  1. hands off until the dashboard says "tared" (let the sensors warm up 20-30 min first)
  2. [s] record 3-5 min: slowly push every axis towards the ORANGE line in + and -,
     never past RED (window 2). Low and high on the handle, down/up, twist, rock, hands off.
  3. [s] stop. Record a second session the same way. When the excitation says "ready", [f] fit.
  4. window 3: calibrated HEX against the reference, axis by axis

 active calibration:   $HEXCAL_PKG/cfg/hex_calibration.yaml
 runs (recordings):    $HEXCAL_PKG/calibration/<date_time>/
 settings:             $HEXCAL_PARAMS
 later, on its own:    roslaunch optoforce_sensor hex_calibrated.launch
 stop everything:      tmux kill-session -t hex_calibration
HELP
  history -s "tmux kill-session -t hex_calibration"
  history -s "roslaunch optoforce_sensor hex_calibrated.launch"
  history -s "rosservice call /hex_calibration_monitor/zero"
  history -s "rosservice call /hex_calibration/fit"
  history -s "rosservice call /hex_calibration/stop"
  history -s "rosservice call /hex_calibration/start"
}
