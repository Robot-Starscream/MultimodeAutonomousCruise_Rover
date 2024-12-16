/*
 * MyMap.h
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#ifndef _MY_MAP_H_
#define _MY_MAP_H_

#include<iostream>
#include<vector>

using namespace std;

struct NewMap
{
    double position[2]; // The position of each grid in the map in the world coordinate system
    float height; // The height value in each grid of the map
    int label; // The label value in each grid of the map
    int index[2]; // The index of each grid in the map, same as the original map
};

struct ObstacleMap
{
    vector< vector<NewMap> > subMap; // The sub-map where the obstacle is located
    double length; // The length of the obstacle relative to the trajectory
    double wide; // The width of the obstacle relative to the trajectory
    float height; // The height of the obstacle
    double distanceWithPath; // The relative distance between the obstacle and the trajectory
    double centerPosition[2]; // The position of the obstacle center in the world coordinate system
    double distanceWithRobotX;// The distance between the obstacle and the robot
    double distanceWithRobotY;
    int label; // The label value of the obstacle
    double angleIN; // Entry angle, exit angle
    double angleOUT;

    string positionRelation; // The positional relationship relative to the trajectory
    string typeOfObstacle; // The type of obstacle
    string modeID; // Motion mode
};

struct ConvolutionMap
{
    float height; // The height value in each grid of the map
    int label; // The label value in each grid of the map
    int index[2]; // The index of each grid in the map, same as the original map
    float convolutionValue;
};

struct Bumps
{
    vector<vector<ConvolutionMap>> subMap;
    float height; // The maximum convolution value of the sharp bump
    int label;
    double radius; // The sharp radius of the sharp bump
    int idex[2]; // The index of the sharp bump on the obstacle surface
    double SN;
    double area;
    double diffHeight;
    int position[2];
};

struct PATH
{
    double relation; // The value obtained after the grey relational analysis method
    int index; // The index of the correlation degree
    vector<double> distance; // The distance between the trajectory and the wheel trajectory, 0-left, 1-right
};

struct OBS
{
    int ID;
    double positionx;
    double positiony;
    double positionz;
};

#endif
