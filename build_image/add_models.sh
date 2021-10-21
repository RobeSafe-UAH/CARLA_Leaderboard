#!/bin/bash

address="www.robesafe.uah.es"

# Perception

scp $1@$address:/home/git/techs4agecar/models/yolov5/yolov5l.pt .tmp
scp $1@$address:/home/git/techs4agecar/models/3d_estimation/epoch_10.pkl .tmp
scp $1@$address:/home/git/techs4agecar/models/3d_estimation/vgg19_bn-c79401a0.pth .tmp