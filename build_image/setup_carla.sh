#!/bin/bash

cd /workspace
git clone -b stable --single-branch https://github.com/carla-simulator/leaderboard.git
git clone -b leaderboard --single-branch https://github.com/carla-simulator/scenario_runner.git