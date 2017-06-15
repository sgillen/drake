#!/bin/bash
set -x -e

# @see https://github.com/RobotLocomotion/drake/issues/5746
# @see https://github.com/RobotLocomotion/spartan/blob/master/scripts/sim_playback.py
./helper_app.sh --script ./sim_playback.py "$@"
