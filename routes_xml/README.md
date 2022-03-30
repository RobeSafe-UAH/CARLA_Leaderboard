This folder contains:

- Routes: Folder with interesting routes designed by the RobeSafe research group. For example, routes_iv_2022_experiment_B_NEAT.xml is a route
ellaborated by the paper "NEAT: Neural Attention Fields for End-to-End Autonomous Driving", interesting to validate the architecture
using different weathers and towns.

- Utils: Folder that contains several useful scripts designed by the RobeSafe research group, overall focused on plotting on the shell
the infractions and writing the results. Our files are _robesafe.

They must placed as following:

	cp leaderboard_evaluator_robesafe.py /workspace/leaderboard/leaderboard_evaluator.py
	cp scenario_manager_robesafe.py /workspace/leaderboard/scenarios/scenario_manager.py
	cp results_writer_robesafe.py /workspace/leaderboard/utils/results_writer.py
	cp statistics_manager_robesafe.py /workspace/leaderboard/statistics_manager.py

	cp atomic_criteria_robesafe.py /workspace/scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_criteria.py

