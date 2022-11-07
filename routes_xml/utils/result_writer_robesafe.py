#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains the result gatherer and write for CARLA scenarios.
It shall be used from the ScenarioManager only.

Modified by RobeSafe research group
"""

from __future__ import print_function

import time
import csv
import os
from tabulate import tabulate
from datetime import datetime

class ResultOutputProvider(object):

    """
    This module contains the _result gatherer and write for CARLA scenarios.
    It shall be used from the ScenarioManager only.
    """

    def __init__(self, data, global_result, town):
        """
        - data contains all scenario-related information
        - global_result is overall pass/fail info
        """
        self._data = data
        self._global_result = global_result
        self.town = town

        self._start_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                         time.localtime(self._data.start_system_time))
        self._end_time = time.strftime('%Y-%m-%d %H:%M:%S',
                                       time.localtime(self._data.end_system_time))

        # Write log file

        self._running_red_lights = 0.0
        self._collisions = 0.0

        print(self.create_output_text())

    def create_output_text(self):
        """
        Creates the output message
        """

        # Create the title
        output = "\n"
        output += "\033[1m========= Results of {} (repetition {}) ------ {} \033[1m=========\033[0m\n".format(
            self._data.scenario_tree.name, self._data.repetition_number, self._global_result)
        output += "\n"

        # Simulation part
        system_time = round(self._data.scenario_duration_system, 2)
        game_time = round(self._data.scenario_duration_game, 2)
        ratio = round(self._data.scenario_duration_game / self._data.scenario_duration_system, 3)

        list_statistics = [["Start Time", "{}".format(self._start_time)]]
        list_statistics.extend([["End Time", "{}".format(self._end_time)]])
        list_statistics.extend([["Duration (System Time)", "{}s".format(system_time)]])
        list_statistics.extend([["Duration (Game Time)", "{}s".format(game_time)]])
        list_statistics.extend([["Ratio (System Time / Game Time)", "{}".format(ratio)]])

        output += tabulate(list_statistics, tablefmt='fancy_grid')
        output += "\n\n"

        # Criteria part
        header = ['Criterion', 'Result', 'Value']
        list_statistics = [header]

        for criterion in self._data.scenario.get_criteria():

            actual_value = criterion.actual_value
            expected_value = criterion.expected_value_success
            name = criterion.name

            result = criterion.test_status

            if result == "SUCCESS":
                result = '\033[92m'+'SUCCESS'+'\033[0m'
            elif result == "FAILURE":
                result = '\033[91m'+'FAILURE'+'\033[0m'

            if name == "RouteCompletionTest":
                self._route_completion = actual_value
                actual_value = str(actual_value) + " %"
            elif name == "OutsideRouteLanesTest":
                self._outside_route = actual_value
                actual_value = str(actual_value) + " %"
            elif name == "CollisionTest":
                self._collisions = actual_value
                actual_value = str(actual_value) + " times"
            elif name == "RunningRedLightTest":
                self._running_red_lights = actual_value
                actual_value = str(actual_value) + " times"
            elif name == "RunningStopTest":
                self._running_stops = actual_value
                actual_value = str(actual_value) + " times"
            elif name == "InRouteTest":
                self._in_route = criterion.test_status
                actual_value = ""
                
            elif name == "AgentBlockedTest":
                self._agent_blocked = criterion.test_status
                actual_value = ""

            list_statistics.extend([[name, result, actual_value]])

        # Timeout
        name = "Timeout"

        actual_value = self._data.scenario_duration_game
        expected_value = self._data.scenario.timeout

        if self._data.scenario_duration_game < self._data.scenario.timeout:
            result = '\033[92m'+'SUCCESS'+'\033[0m'
            self._timeout = "SUCCESS"
        else:
            result = '\033[91m'+'FAILURE'+'\033[0m'
            self._timeout = "FAILURE"

        list_statistics.extend([[name, result, '']])

        output += tabulate(list_statistics, tablefmt='fancy_grid')

        # Save results as .txt
        now = datetime.now()
        now = now.strftime("%Y-%m-%d")
        txt_file = "/workspace/results/log_" + now + ".txt"

        with open(txt_file, "a") as file_object:
            file_object.write(output)

        print("Saved log in: ", txt_file)

        # Save results as .csv
        csv_file = "/workspace/results/log_" + now + ".csv"
        rows = []

        rct_data = []
        orlt_data = []
        ct_data = []
        rrlt_data = []
        rst_data = []
        
        try: 
            with open(csv_file, mode='r') as data:
                csv_reader = csv.reader(data, delimiter = '\t', lineterminator = '\n')
                # extracting field names through first row
                fields_data = next(csv_reader)
                if fields_data[0] != 'Id':
                    print(f"Error in {csv_file}: incorrect file format.")
                else:
                    for row in csv_reader:
                        if row[0] == '-': # Delimiter
                            break
                        else:
                            rows.append(row)
                            rct = float(row[4])
                            rct_data.append(rct)
                            orlt = float(row[5])
                            orlt_data.append(orlt)
                            ct = float(row[6])
                            ct_data.append(ct)
                            rrlt = float(row[7])
                            rrlt_data.append(rrlt)
                            rst = float(row[8])
                            rst_data.append(rst)
                            
        except:
            print(f"{csv_file} file does not exist. Creating file.....")

        id_actual = ''.join(filter(str.isdigit, self._data.scenario_tree.name))
        average_rct = float("{:.3f}".format((sum(rct_data) + self._route_completion) / (len(rct_data) + 1)))
        average_orlt = float("{:.3f}".format((sum(orlt_data) + self._outside_route) / (len(orlt_data) + 1)))
        total_ct = float("{:.3f}".format(sum(ct_data) + self._collisions))
        total_rrlt = float("{:.3f}".format(sum(rrlt_data) + self._running_red_lights))
        total_rst = float("{:.3f}".format(sum(rst_data) + self._running_stops))
        
        fields = ['Id', 'Town', 'DST', 'DGT', 'RCT', 'ORLT', 'CT', 'RRLT', 'RST', 'IRT', 'ABT', 'TOut']

        aux = [id_actual, str(self.town), str(system_time), str(game_time), str(self._route_completion), 
                str(self._outside_route), str(self._collisions), str(self._running_red_lights),
                str(self._running_stops), self._in_route[0], self._agent_blocked[0], self._timeout[0]]
        rows.append(aux)
        aux = ['-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-']
        rows.append(aux)
        aux = ['', '', '', '', str(average_rct), str(average_orlt), str(total_ct), str(total_rrlt), str(total_rst), '', '', '']
        rows.append(aux)

        with open(csv_file, mode='w') as data:
            csv_writer = csv.writer(data, delimiter = '\t', lineterminator = '\n')
            csv_writer.writerow(fields)
            csv_writer.writerows(rows)

        print("Saved log in: ", csv_file)
      
        return output
