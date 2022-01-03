#!/bin/bash

echo 'y' | rosnode cleanup
rosnode list | grep -v rosout/ | xargs rosnode kill