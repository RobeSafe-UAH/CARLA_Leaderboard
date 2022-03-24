#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module contains a statistics manager for the CARLA AD leaderboard
"""

from __future__ import print_function

from dictor import dictor
import math
import sys
from datetime import datetime
import csv

from srunner.scenariomanager.traffic_events import TrafficEventType

from leaderboard.utils.checkpoint_tools import fetch_dict, save_dict, create_default_json_msg

PENALTY_COLLISION_PEDESTRIAN = 0.50
PENALTY_COLLISION_VEHICLE = 0.60
PENALTY_COLLISION_STATIC = 0.65
PENALTY_TRAFFIC_LIGHT = 0.70
PENALTY_STOP = 0.80


class RouteRecord():
    def __init__(self):
        self.route_id = None
        self.index = None
        self.status = 'Started'
        self.infractions = {
            'collisions_pedestrian': [],
            'collisions_vehicle': [],
            'collisions_layout': [],
            'red_light': [],
            'stop_infraction': [],
            'outside_route_lanes': [],
            'route_dev': [],
            'route_timeout': [],
            'vehicle_blocked': []
        }

        self.scores = {
            'score_route': 0,
            'score_penalty': 0,
            'score_composed': 0
        }

        self.meta = {}


def to_route_record(record_dict):
    record = RouteRecord()
    for key, value in record_dict.items():
        setattr(record, key, value)

    return record


def compute_route_length(config):
    trajectory = config.trajectory

    route_length = 0.0
    previous_location = None
    for location in trajectory:
        if previous_location:
            dist = math.sqrt((location.x-previous_location.x)*(location.x-previous_location.x) +
                             (location.y-previous_location.y)*(location.y-previous_location.y) +
                             (location.z - previous_location.z) * (location.z - previous_location.z))
            route_length += dist
        previous_location = location

    return route_length


class StatisticsManager(object):

    """
    This is the statistics manager for the CARLA leaderboard.
    It gathers data at runtime via the scenario evaluation criteria.
    """

    def __init__(self):
        self._master_scenario = None
        self._registry_route_records = []

    def resume(self, endpoint):
        data = fetch_dict(endpoint)

        if data and dictor(data, '_checkpoint.records'):
            records = data['_checkpoint']['records']

            for record in records:
                self._registry_route_records.append(to_route_record(record))

    def set_route(self, route_id, index):

        self._master_scenario = None
        route_record = RouteRecord()
        route_record.route_id = route_id
        route_record.index = index

        if index < len(self._registry_route_records):
            # the element already exists and therefore we update it
            self._registry_route_records[index] = route_record
        else:
            self._registry_route_records.append(route_record)

    def set_scenario(self, scenario):
        """
        Sets the scenario from which the statistics will be taken.
        
        This works in conjunction with set_route so that the variable
        is only active when the simulation is active, to avoid statistic
        errors in case something breaks between simulations 
        """
        self._master_scenario = scenario

    def compute_route_statistics(self, config, duration_time_system=-1, duration_time_game=-1, failure=""):
        """
        Compute the current statistics by evaluating all relevant scenario criteria
        """
        index = config.index

        if not self._registry_route_records or index >= len(self._registry_route_records):
            raise Exception('Critical error with the route registry.')

        # fetch latest record to fill in
        route_record = self._registry_route_records[index]

        target_reached = False
        score_penalty = 1.0
        score_route = 0.0
        driving_score = 0.0
        collisions_pedestrian = 0
        collisions_vehicle = 0
        collisions_layout = 0
        red_light = 0
        stop_infraction = 0
        outside_route_lanes = 0
        route_dev = 0
        route_timeout = 0
        vehicle_blocked = 0

        route_length = compute_route_length(config)

        route_record.meta['duration_system'] = duration_time_system
        route_record.meta['duration_game'] = duration_time_game
        route_record.meta['route_length'] = route_length

        if self._master_scenario:
            if self._master_scenario.timeout_node.timeout:
                route_record.infractions['route_timeout'].append('Route timeout.')
                failure = "Agent timed out"

            for node in self._master_scenario.get_criteria():
                if node.list_traffic_events:
                    # analyze all traffic events
                    for event in node.list_traffic_events:
                        if event.get_type() == TrafficEventType.COLLISION_STATIC:
                            score_penalty *= PENALTY_COLLISION_STATIC
                            route_record.infractions['collisions_layout'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.COLLISION_PEDESTRIAN:
                            score_penalty *= PENALTY_COLLISION_PEDESTRIAN
                            route_record.infractions['collisions_pedestrian'].append(event.get_message())
                            
                        elif event.get_type() == TrafficEventType.COLLISION_VEHICLE:
                            score_penalty *= PENALTY_COLLISION_VEHICLE
                            route_record.infractions['collisions_vehicle'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION:
                            score_penalty *= (1 - event.get_dict()['percentage'] / 100)
                            route_record.infractions['outside_route_lanes'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.TRAFFIC_LIGHT_INFRACTION:
                            score_penalty *= PENALTY_TRAFFIC_LIGHT
                            route_record.infractions['red_light'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.ROUTE_DEVIATION:
                            route_record.infractions['route_dev'].append(event.get_message())
                            failure = "Agent deviated from the route"

                        elif event.get_type() == TrafficEventType.STOP_INFRACTION:
                            score_penalty *= PENALTY_STOP
                            route_record.infractions['stop_infraction'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.VEHICLE_BLOCKED:
                            route_record.infractions['vehicle_blocked'].append(event.get_message())
                            failure = "Agent got blocked"

                        elif event.get_type() == TrafficEventType.ROUTE_COMPLETED:
                            score_route = 100.0
                            target_reached = True
                        elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                            if not target_reached:
                                if event.get_dict():
                                    score_route = event.get_dict()['route_completed']
                                else:
                                    score_route = 0

        # update route scores
        route_record.scores['score_route'] = score_route
        route_record.scores['score_penalty'] = score_penalty
        driving_score = max(score_route*score_penalty, 0.0)
        route_record.scores['score_composed'] = driving_score

        # update status
        if target_reached:
            route_record.status = 'Completed'
        else:
            route_record.status = 'Failed'
            if failure:
                route_record.status += ' - ' + failure

        # Save results as .txt
        now = datetime.now()
        now = now.strftime("%Y-%m-%d")
        txt_file = "/workspace/results/log_" + now + ".txt"
    
        output = "\n\n"
        output += "------------------------ROUTE_RECORD: -------------------------\n"
        output += "*** Route_id:" + str(route_record.route_id) + "\n"
        output += "*** Index:" + str(route_record.index) + "\n"
        output += "*** Status:" + str(route_record.status) + "\n"
        output += "*** Infractions:" + str(route_record.infractions) + "\n"
        output += "*** Scores:" + str(route_record.scores) + "\n"
        output += "*** Meta:" + str(route_record.meta) + "\n"
        output += "---------------------------------------------------------------\n"

        with open(txt_file, "a") as file_object:
            file_object.write(output)
        
        print("Saveing statistics in: ", txt_file)

        # Save results as .csv
        
        csv_file = "/workspace/results/log_statistics_" + now + ".csv"
        rows = []

        total_rl = 0.0
        total_ds = 0.0
        total_rc = 0.0
        total_ip = 0.0
        total_cp = 0
        total_cv = 0
        total_cl = 0
        total_rli = 0
        total_ssi = 0
        total_ori = 0
        total_rd = 0
        total_rt = 0
        total_ab = 0
        
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
                            total_rl = total_rl + float(row[2])
                            total_ds = total_ds + float(row[5])
                            total_rc = total_rc + float(row[6])
                            total_ip = total_ip + float(row[7])
                            total_cp = total_cp + int(row[8])
                            total_cv = total_cv + int(row[9])
                            total_cl = total_cl + int(row[10])
                            total_rli = total_rli + int(row[11])
                            total_ssi = total_ssi + int(row[12])
                            total_ori = total_ori + int(row[13])
                            total_rd = total_rd + int(row[14])
                            total_rt = total_rt + int(row[15])
                            total_ab = total_ab + int(row[16])

        except:
            print(f"{csv_file} file does not exist. Creating file.....")

        id_actual = ''.join(filter(str.isdigit, route_record.route_id))

        n_data = (len(rows) + 1)
        
        route_length = float("{:.2f}".format(route_length))

        duration_time_system = float("{:.2f}".format(duration_time_system))
        duration_time_game = float("{:.2f}".format(duration_time_game))
        driving_score = float("{:.3f}".format(driving_score))
        score_route = float("{:.3f}".format(score_route))
        score_penalty = float("{:.3f}".format(score_penalty))

        average_ds = float("{:.3f}".format((total_ds + driving_score) / n_data))
        average_rc = float("{:.3f}".format((total_rc + score_route) / n_data))
        average_ip = float("{:.3f}".format((total_ip + score_penalty) / n_data))

        total_rl = total_rl + route_length
        total_rl_kms = max(average_rc / 100 * total_rl / 1000.0, 0.001)

        collisions_pedestrian = len(route_record.infractions['collisions_pedestrian'])
        total_cp = float("{:.3f}".format((total_cp + collisions_pedestrian)/total_rl_kms)) # infraction/km
        collisions_vehicle = len(route_record.infractions['collisions_vehicle'])
        total_cv = float("{:.3f}".format((total_cv + collisions_vehicle)/total_rl_kms))
        collisions_layout = len(route_record.infractions['collisions_layout'])
        total_cl = float("{:.3f}".format((total_cl + collisions_layout)/total_rl_kms))
        red_light = len(route_record.infractions['red_light'])
        total_rli = float("{:.3f}".format((total_rli + red_light)/total_rl_kms))
        stop_infraction = len(route_record.infractions['stop_infraction'])
        total_ssi = float("{:.3f}".format((total_ssi + stop_infraction)/total_rl_kms))
        outside_route_lanes = len(route_record.infractions['outside_route_lanes'])
        total_ori = float("{:.3f}".format((total_ori + outside_route_lanes)/total_rl_kms))
        route_dev = len(route_record.infractions['route_dev'])
        total_rd = float("{:.3f}".format((total_rd + route_dev)/total_rl_kms))
        route_timeout = len(route_record.infractions['route_timeout'])
        total_rt = float("{:.3f}".format((total_rt + route_timeout)/total_rl_kms))
        vehicle_blocked = len(route_record.infractions['vehicle_blocked'])
        total_ab = float("{:.3f}".format((total_ab + vehicle_blocked)/total_rl_kms))

        fields = ['Id', 'Town', 'RL', 'DST', 'DGT', 'DS', 'RC', 'IP', 'CP', 'CV', 'CL', 'RLI', 'SSI', 'ORI', 'RD', 'RT', 'AB']

        aux = [id_actual, config.town,  str(route_length), str(duration_time_system), str(duration_time_game), str(driving_score), 
                str(score_route), str(score_penalty), str(collisions_pedestrian), str(collisions_vehicle), str(collisions_layout),
                str(red_light), str(stop_infraction), str(outside_route_lanes), str(route_dev), str(route_timeout), str(vehicle_blocked)]
        rows.append(aux)
        aux = ['-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-', '-']
        rows.append(aux)
        aux = ['', '', '', '', '', str(average_ds), str(average_rc), str(average_ip), str(total_cp), str(total_cv), str(total_cl), 
                str(total_rli), str(total_ssi), str(total_ori), str(total_rd), str(total_rt), str(total_ab)]
        rows.append(aux)

        with open(csv_file, mode='w') as data:
            csv_writer = csv.writer(data, delimiter = '\t', lineterminator = '\n')
            csv_writer.writerow(fields)
            csv_writer.writerows(rows)

        print("Saved statistics in: ", csv_file)

        return route_record

    def compute_global_statistics(self, total_routes):
        global_record = RouteRecord()
        global_record.route_id = -1
        global_record.index = -1
        global_record.status = 'Completed'
        global_record.scores_std_dev = RouteRecord().scores

        if self._registry_route_records:
            for route_record in self._registry_route_records:
                global_record.scores['score_route'] += route_record.scores['score_route']
                global_record.scores['score_penalty'] += route_record.scores['score_penalty']
                global_record.scores['score_composed'] += route_record.scores['score_composed']

                for key in global_record.infractions.keys():
                    route_length_kms = max(route_record.scores['score_route'] / 100 * route_record.meta['route_length'] / 1000.0, 0.001)
                    if isinstance(global_record.infractions[key], list):
                        global_record.infractions[key] = len(route_record.infractions[key]) / route_length_kms
                    else:
                        global_record.infractions[key] += len(route_record.infractions[key]) / route_length_kms

                if route_record.status is not 'Completed':
                    global_record.status = 'Failed'
                    if 'exceptions' != global_record.meta:
                        global_record.meta['exceptions'] = []
                    global_record.meta['exceptions'].append((route_record.route_id,
                                                             route_record.index,
                                                             route_record.status))

            for key in global_record.scores.keys():
                global_record.scores[key] /= float(total_routes)

            if total_routes == 1:
                for key in global_record.scores_std_dev.keys():
                    global_record.scores_std_dev[key] = 'NaN'
            else:
                for route_record in self._registry_route_records:
                    for key in global_record.scores_std_dev.keys():
                        global_record.scores_std_dev[key] += math.pow(route_record.scores[key] - global_record.scores[key], 2)

                for key in global_record.scores_std_dev.keys():
                    global_record.scores_std_dev[key] = math.sqrt(global_record.scores_std_dev[key] / float(total_routes - 1))

        # Save results as .txt
        now = datetime.now()
        now = now.strftime("%Y-%m-%d")
        txt_file = "/workspace/results/log_" + now + ".txt"

        output = "\n\n"
        output += "------------------------GLOBAL_ROUTE: -------------------------\n"
        output += "*** Route_id:" + str(global_record.route_id) + "\n"
        output += "*** Index:" + str(global_record.index) + "\n"
        output += "*** Status:" + str(global_record.status) + "\n"
        output += "*** Infractions:" + str(global_record.infractions) + "\n"
        output += "*** Scores:" + str(global_record.scores) + "\n"
        output += "*** Meta:" + str(global_record.meta) + "\n"
        output += "---------------------------------------------------------------\n"

        with open(txt_file, "a") as file_object:
            file_object.write(output)
        
        print("Saved global statistics in: ", txt_file)
        
        return global_record

    @staticmethod
    def save_record(route_record, index, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        record_list = data['_checkpoint']['records']
        if index > len(record_list):
            print('Error! No enough entries in the list')
            sys.exit(-1)
        elif index == len(record_list):
            record_list.append(stats_dict)
        else:
            record_list[index] = stats_dict

        save_dict(endpoint, data)

    @staticmethod
    def save_global_record(route_record, sensors, total_routes, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        data['_checkpoint']['global_record'] = stats_dict
        data['values'] = ['{:.3f}'.format(stats_dict['scores']['score_composed']),
                          '{:.3f}'.format(stats_dict['scores']['score_route']),
                          '{:.3f}'.format(stats_dict['scores']['score_penalty']),
                          # infractions
                          '{:.3f}'.format(stats_dict['infractions']['collisions_pedestrian']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_vehicle']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_layout']),
                          '{:.3f}'.format(stats_dict['infractions']['red_light']),
                          '{:.3f}'.format(stats_dict['infractions']['stop_infraction']),
                          '{:.3f}'.format(stats_dict['infractions']['outside_route_lanes']),
                          '{:.3f}'.format(stats_dict['infractions']['route_dev']),
                          '{:.3f}'.format(stats_dict['infractions']['route_timeout']),
                          '{:.3f}'.format(stats_dict['infractions']['vehicle_blocked'])
                          ]

        data['labels'] = ['Avg. driving score',
                          'Avg. route completion',
                          'Avg. infraction penalty',
                          'Collisions with pedestrians',
                          'Collisions with vehicles',
                          'Collisions with layout',
                          'Red lights infractions',
                          'Stop sign infractions',
                          'Off-road infractions',
                          'Route deviations',
                          'Route timeouts',
                          'Agent blocked'
                          ]

        entry_status = "Finished"
        eligible = True

        route_records = data["_checkpoint"]["records"]
        progress = data["_checkpoint"]["progress"]

        if progress[1] != total_routes:
            raise Exception('Critical error with the route registry.')

        if len(route_records) != total_routes or progress[0] != progress[1]:
            entry_status = "Finished with missing data"
            eligible = False
        else:
            for route in route_records:
                route_status = route["status"]
                if "Agent" in route_status:
                    entry_status = "Finished with agent errors"
                    break

        data['entry_status'] = entry_status
        data['eligible'] = eligible

        save_dict(endpoint, data)

    

    @staticmethod
    def save_sensors(sensors, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        if not data['sensors']:
            data['sensors'] = sensors

            save_dict(endpoint, data)

    @staticmethod
    def save_entry_status(entry_status, eligible, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        data['entry_status'] = entry_status
        data['eligible'] = eligible
        save_dict(endpoint, data)

    @staticmethod
    def clear_record(endpoint):
        if not endpoint.startswith(('http:', 'https:', 'ftp:')):
            with open(endpoint, 'w') as fd:
                fd.truncate(0)
