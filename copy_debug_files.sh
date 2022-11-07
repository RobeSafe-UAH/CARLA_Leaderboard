#!/bin/bash

# Carlos Gómez-Huélamo

# File to substitute some files of the leaderboard or scenario_runner
# folders to debug our Autonomous Driving Stack. In our own files, we have
# included some utilities in order to obtain the different infractions, scores,
# etc. both as .csv files and printing these values on the terminal.

# The original files are:

# /workspace/leaderboard/leaderboard/scenarios/scenario_manager.py 
# /workspace/leaderboard/leaderboard/utils/result_writer.py 
# /workspace/leaderboard/leaderboard/utils/statistics_manager.py 
# /workspace/leaderboard/leaderboard_evaluator.py 
# /workspace/scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_criteria.py 

# E.g. cd /workspace/team_code && ./copy_debug_files.sh debug_local (if you are validiting locally)
#                                                       or leaderboard_cloud (if you are going to submit to the cloud)
# N.B. Make sure this file has execution permits (chmod +x copy_debug_files.sh)

if [[ $1 == "debug_local" ]] # Replace with your modified files
then
    cp routes_xml/utils/scenario_manager_robesafe.py /workspace/leaderboard/leaderboard/scenarios/scenario_manager.py
    cp routes_xml/utils/result_writer_robesafe.py /workspace/leaderboard/leaderboard/utils/result_writer.py 
    cp routes_xml/utils/statistics_manager_robesafe.py /workspace/leaderboard/leaderboard/utils/statistics_manager.py 
    cp routes_xml/utils/leaderboard_evaluator_robesafe.py /workspace/leaderboard/leaderboard/leaderboard_evaluator.py 
    cp routes_xml/utils/atomic_criteria_robesafe.py /workspace/scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_criteria.py
    echo "Replace with our debug files"

elif [[ $1 == "leaderboard_cloud" ]] # Replace with the original files
then
    cp routes_xml/utils/scenario_manager_original.py /workspace/leaderboard/leaderboard/scenarios/scenario_manager.py
    cp routes_xml/utils/result_writer_original.py /workspace/leaderboard/leaderboard/utils/result_writer.py 
    cp routes_xml/utils/statistics_manager_original.py /workspace/leaderboard/leaderboard/utils/statistics_manager.py 
    cp routes_xml/utils/leaderboard_evaluator_original.py /workspace/leaderboard/leaderboard/leaderboard_evaluator.py 
    cp routes_xml/utils/atomic_criteria_original.py /workspace/scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_criteria.py
    echo "Replace with the original files"
fi