#!/bin/bash

address="www.robesafe.uah.es"
yolov5_folder=".tmp/catkin_ws/src/t4ac_unified_perception_layer/models/yolov5"
estimation_3d_folder=".tmp/catkin_ws/src/t4ac_unified_perception_layer/models/3d_estimation"

# Perception

if [ -d $yolov5_folder ] 
then 
    continue
else 
    mkdir $yolov5_folder
fi
scp $1@$address:/home/git/techs4agecar/models/yolov5/yolov5l.pt $yolov5_folder/yolov5l.pt

if [ -d $estimation_3d_folder ] 
then 
    continue
else 
    mkdir $estimation_3d_folder
fi

scp $1@$address:/home/git/techs4agecar/models/3d_estimation/epoch_10.pkl $estimation_3d_folder/epoch_10.pkl
scp $1@$address:/home/git/techs4agecar/models/3d_estimation/vgg19_bn-c79401a0.pth $estimation_3d_folder/vgg19_bn-c79401a0.pth