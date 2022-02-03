#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
Created on tue Oct 26 16:32:09 2021
@author: Carlos Gómez Huélamo
"""

import pandas as pd
import csv
import glob
import glob2
import copy
import os
import argparse
import re

def safe_path(input_path):
    """
    """
    safe_data = copy.copy(input_path)
    safe_data = os.path.normpath(safe_data)
    return safe_data

def isstring(string_test):
    """
    """
    return isinstance(string_test, str)

def load_list_from_folder(folder_path, ext_filter=None, depth=1, recursive=False, sort=True, save_path=None):
    """
    """
    folder_path = safe_path(folder_path)
    if isstring(ext_filter): ext_filter = [ext_filter]

    full_list = []
    if depth is None: # Find all files recursively
        recursive = True
        wildcard_prefix = '**'
        if ext_filter is not None:
            for ext_tmp in ext_filter:
                wildcard = os.path.join(wildcard_prefix,'*'+ext_tmp)
                curlist = glob2.glob(os.path.join(folder_path,wildcard))
                if sort: curlist = sorted(curlist)
                full_list += curlist
        else:
            wildcard = wildcard_prefix
            curlist = glob2.glob(os.path.join(folder_path, wildcard))
            if sort: curlist = sorted(curlist)
            full_list += curlist
    else: # Find files based on depth and recursive flag
        wildcard_prefix = '*'
        for index in range(depth-1): wildcard_prefix = os.path.join(wildcard_prefix, '*')
        if ext_filter is not None:
            for ext_tmp in ext_filter:
                wildcard = wildcard_prefix + ext_tmp
                curlist = glob.glob(os.path.join(folder_path, wildcard))
                if sort: curlist = sorted(curlist)
                full_list += curlist
        else:
            wildcard = wildcard_prefix
            curlist = glob.glob(os.path.join(folder_path, wildcard))
            if sort: curlist = sorted(curlist)
            full_list += curlist
        if recursive and depth > 1:
            newlist, _ = load_list_from_folder(folder_path=folder_path, ext_filter=ext_filter, depth=depth-1, recursive=True)
            full_list += newlist

    full_list = [os.path.normpath(path_tmp) for path_tmp in full_list]
    num_elem = len(full_list)

    return full_list, num_elem

def calculate_metrics(pd_frame, metrics_names, min_metric):
    metrics = {}
    # "DS", "RC", "IP", "CP", "CV", "CL", "RLI", "SSI", "ORI", "RD", "RT", "AB"
    rl_sum = pd_frame["RL"].astype("float").sum() / 1000.0
    for i, mn in enumerate(metrics_names):
        if i < min_metric:
            metrics[mn] = round(pd_frame[mn].astype("float").mean(), 2)
        else:
            metrics[mn] = round(pd_frame[mn].astype("float").sum() / rl_sum, 2)
    return metrics

def calculate_metrics_per_town(df, metrics_names, min_metric):
    towns = df["Town"].unique()
    new_df = None
    cp_idx = metrics_names.index("CP")
    for i, town in enumerate(towns):
        pd_town = df.loc[df["Town"] == town]
        metrics = {"Town": town}
        metrics.update(calculate_metrics(pd_town, metrics_names, min_metric))
        if new_df is None:
            new_df = pd.DataFrame(metrics, index=[i], dtype=object)
        else:
            new_df = new_df.append(metrics, ignore_index=True)
    metrics = calculate_metrics(new_df, metrics_names, min_metric)
    new_df = new_df.append(metrics, ignore_index=True)
    return new_df

if __name__ == "__main__":

    ## parser
    parser = argparse.ArgumentParser()
    parser.add_argument("-exp", "--experiment", type=str, required=True)
    parser.add_argument("-of", "--output_folder", type=str, required=False)
    args = vars(parser.parse_args())
    exp_folder = args["experiment"]
    out_folder = args["output_folder"] if args["output_folder"] is not None else args["experiment"]

    ## load files
    files, num_files = load_list_from_folder(exp_folder)
    df_merged = [] # Only the file with statistics

    r = re.compile("Town*") # search Town in dataframe. Filter the outliers
    for file_str in files:
        if "statistics" in file_str and ".csv" in file_str:
            df = pd.read_csv(file_str,delim_whitespace=True)
            df_towns = list(df["Town"])
            idx = len(list(filter(r.match, df_towns)))
            df_rows = df[:idx] # Avoid the last N (take only until the last town)
            df_merged.append(df_rows)

    # merge csv
    if len(df_merged) == 0:
        print("Files not found")
    if len(df_merged) > 1:
        df_merged = pd.concat(df_merged).reset_index(drop=True).drop(columns=["Id"])
    else:
        df_merged = df_merged[0]
    
    ## metrics
    metric_names = ["RL", "DS", "RC", "IP", "CP", "CV", "CL", "RLI", "SSI", "ORI", "RD", "RT", "AB"]
    cp_idx = metric_names.index("CP")

    # town metrics
    town_metrics = calculate_metrics_per_town(df_merged.copy(), metric_names, cp_idx)

    # global metrics
    metrics = calculate_metrics(df_merged.copy(), metric_names, cp_idx)
    for i in range(cp_idx, len(metric_names)):
        df_merged[metric_names[i]] = df_merged[metric_names[i]].astype(str)
    global_metrics = df_merged.append(metrics, ignore_index=True)

    # save data
    start_inference_data = out_folder.split('/')[-1]
    town_metrics.to_csv(os.path.join(out_folder, "town_metrics_"+start_inference_data+".csv"), index_label="Id", sep="\t", na_rep="-", float_format="%.3f")
    global_metrics.to_csv(os.path.join(out_folder, "global_metrics_"+start_inference_data+".csv"), index_label="Id", sep="\t", na_rep="-", float_format="%.3f")