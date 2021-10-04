FROM nvidia/cuda:11.2.2-devel-ubuntu20.04

ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG http_proxy

RUN DEBIAN_FRONTEND="noninteractive" apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install --reinstall -y locales && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US
ENV LC_ALL en_US.UTF-8

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
         apt-utils \
         build-essential \
         cmake \
         git \
         curl \
         htop \
         vim \
         ca-certificates \
         libjpeg-dev \
	     libpng16-16 \
	     libtiff5 \
         libpng-dev \
         lsb-core \
         python3-dev \
         python3-setuptools \
         python3-pip && \ 
         pip3 install --upgrade pip && \
         rm -rf /var/lib/apt/lists/*

## Install ros
WORKDIR /workspace/
ADD install_ros.sh /workspace/install_ros.sh
RUN chmod +x install_ros.sh && sh install_ros.sh

## Install Python libraries
ADD requirements.txt /workspace/requirements.txt
RUN pip3 install -r requirements.txt

## Setup global ENV variables
ADD PythonAPI /workspace/CARLA/PythonAPI
RUN cd /workspace/CARLA/PythonAPI/carla/dist && \
    mv carla-0.9.10-py3.7-linux-x86_64.egg carla-leaderboard-py3x.egg
ENV CARLA_ROOT="/workspace/CARLA" SCENARIO_RUNNER_ROOT="/workspace/scenario_runner" \
    LEADERBOARD_ROOT="/workspace/leaderboard" TEAM_CODE_ROOT="/workspace/team_code" \
    PYTHONPATH="/workspace/CARLA/PythonAPI/carla/dist/carla-leaderboard-py3x.egg:/workspace/scenario_runner:/workspace/CARLA/PythonAPI/carla:/workspace/leaderboard:${PYTHONPATH}" \
    PATH="/workspace/CARLA/PythonAPI/carla/dist/carla-leaderboard-py3x.egg:$PATH"
ENV TEAM_AGENT="${TEAM_CODE_ROOT}/RobesafeAgent.py" CHALLENGE_TRACK_CODENAME="MAP" \
    SCENARIOS="${LEADERBOARD_ROOT}/data/all_towns_traffic_scenarios_public.json" \
    ROUTES="${LEADERBOARD_ROOT}/data/routes_training.xml" REPETITIONS="1" \
    CHECKPOINT_ENDPOINT="/workspace/results/results.json" DEBUG_CHALLENGE="0" \
    HTTP_PROXY="" HTTPS_PROXY="" http_proxy="" https_proxy=""

## Setup CARLA
ADD setup_carla.sh /workspace/
RUN chmod +x setup_carla.sh && sh setup_carla.sh

## Give permissions
RUN mkdir -p /workspace/results
RUN chmod +x /workspace/leaderboard/scripts/run_evaluation.sh

## Setup ROS
RUN mkdir -p /workspace/team_code/catkin_ws/src
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash;\
    cd /workspace/team_code/catkin_ws;\
    catkin_make -j8;\
    echo "source /workspace/team_code/catkin_ws/devel/setup.bash" >> ~/.bashrc'

## Entrypoint
ADD agent_sources.sh /root/
RUN chmod +x /root/agent_sources.sh

WORKDIR /workspace/

CMD ["/bin/bash"]
