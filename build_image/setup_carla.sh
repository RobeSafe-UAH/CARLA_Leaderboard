#!/bin/bash

git clone -b stable --single-branch https://github.com/carla-simulator/leaderboard.git
# cd $LEADERBOARD_ROOT && pip3 install -r requirements.txt
git clone -b leaderboard --single-branch https://github.com/carla-simulator/scenario_runner.git
# cd $SCENARIO_RUNNER_ROOT && pip3 install -r requirements.txt
cd /workspace