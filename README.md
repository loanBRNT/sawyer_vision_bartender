# sawyer_vision_bartender

This repository is a part of my intenrship at the Human Centered Robotic Laboratory at the University of Applied Science of Ostfalia. The purpose is to realize a bartender application for the sawyer Robot from Rethink Robotic. The global repository is confidential but this is my datassets, scripts and model that I used to develop the detection part.

## Models and performances


## Versions and Videos

**V1.0** | 01/06 | : Fixed Position, Bottle already opened --> [video](https://youtu.be/lT4WaLM3AWw)

**V1.5** | 10/06 | : Fixed Position, Bottle already opened --> [video](https://youtu.be/lT4WaLM3AWw)

**V2.0** | 03/07 | : Detection, Bottle already opened --> [video](https://youtu.be/lT4WaLM3AWw)

## How to use this repository

I created this with an Ubuntu 20.04 virtual machine and ros noetic installed. 

You can clone it and change the `intera.sh` file with your own environment's variable (ROBOT_IP and HOST_IP). Then, you can run followings commands :
- `./intera.sh`
- `catkin_make`

After this, you should be able to use my scripts by calling my bartender_sawyer package.

**Ps :** You can also just add the `bartender_sawyer` folder to your existant project.
