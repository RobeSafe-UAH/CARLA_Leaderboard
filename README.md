The CARLA Autonomous Driving Challenge 2021 is organized as part of the Machine Learning for Autonomous Driving Workshop at NeurIPS 2021. This competition is open to any participant from academia and industry. You only need to sign up on the CARLA AD Leaderboard, providing your team name and your institution.

Timeline
Challenge opening: September 15th, 2021
Challenge closure: November 19th, 2021
Winners of each track are notified: November 22nd, 2021
Video presentation submission: December 1st, 2021
Results presented at ML4AD workshop: December 13th, 2021

**How to clone this repository?**
cd /workspace (in the docker image)
git clone --recursive -b 2021_carla_challenge git@github.com:RobeSafe-UAH/CARLA_Leaderboard.git team_code 

**How to add a submodule?**

git submodule add your_ssh (make sure you got the ssh keys)
e.g. cd /workspace/team_code/catkin_ws/src/ && git submodule add -b 2021_carla_challenge git@github.com:RobeSafe-UAH/Techs4AgeCar-Mapping-Layer.git
