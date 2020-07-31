# Occupancy Grid
This project generates an occupancy map from given sensor readings. The program generates an image with points on three possible colors:
 - unknown state in green
 - obstacle found, in black 
 - empty space, in red.

## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## General info
This project is part of the Udacity Robot Software Engineer nanodegree program Part 05, Lesson 02. 
 
	
## Technologies
Project is created with:
* c++
* ROS (Robot Operating System)
	
## Setup
To run this project: 
1. compile the program
```
$ cd RoboND-OccupancyGridMappingAlgorithm/
$ rm -rf Images/* #Delete the folder content and not the folder itself!
$ g++ main.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
```

2. Finally run the program
```
$ ./app
```
If you get a warning regarding the matplotlib library, just ignore it.

3. Now, wait for the program to generate the map and store it in the /home/workspace/RoboND-OccupancyGridMappingAlgorithm/Images directory!
