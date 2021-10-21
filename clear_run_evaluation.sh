#!/bin/bash

# Carlos Gómez-Huélamo - November 2020

killall -9 rosmaster

var1=$(pgrep -x "run_evaluation.")
var2=$(pgrep -x "python3")
var3=$(pgrep -x "roslaunch")
var4=$(pgrep -x "rosnode")
var5=$(pgrep -x "sh")
var6=$(pgrep -x "rviz")
var7=$(pgrep "darknet")
var8=$(pgrep "t4ac_contr")
var9=$(pgrep "graphics_node")
var10=$(pgrep "ekf_loc")
var11=$(pgrep "sec_tf")
var12=$(pgrep "python")
var13=$(pgrep "rosout")
var14=$(pgrep "elas_ros")
var15=$(pgrep "bev_from_2d_obj")
var16=$(pgrep "image_proc")
var17=$(pgrep "rostopic")
var18=$(pgrep "rqt_graph")
var19=$(pgrep "sensor_f")
var20=$(pgrep "t4ac")
var21=$(pgrep "dbus")

var="$var1 $var2 $var3 $var4 $var5 $var6 $var7 $var8 $var9 $var10 
     $var11 $var12 $var13 $var14 $var15 $var16 $var17 $var18 $var19 $var20 $var21"

for i in $var;
do
    echo $i
    kill -9 ${i}; 
done