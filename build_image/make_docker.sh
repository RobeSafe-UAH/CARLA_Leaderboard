#!/bin/bash

rm -rf .tmp
mkdir -p .tmp
cd .. && rsync -Rr . ./build_image/.tmp
cd ./build_image/

# set --force-rm if you want to build the image from scratch, removing intermediate container
docker build -t $1 . 

rm -rf .tmp