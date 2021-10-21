#!/bin/bash

rm -rf .tmp
mkdir -p .tmp
cd .. && rsync --exclude build_image/.tmp -Rr . ./build_image/.tmp
cd build_image

# Add models

current_folder=$PWD
source $current_folder/add_models.sh $2

# set --force-rm if you want to build the image from scratch, removing intermediate container
docker build -t $1 . 

rm -rf .tmp
