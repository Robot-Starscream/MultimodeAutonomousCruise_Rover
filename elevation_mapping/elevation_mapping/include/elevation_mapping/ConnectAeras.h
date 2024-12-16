/*
 * ConnectAeras.h
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#ifndef _CONNECTAERAS_H
#define _CONNECTAERAS_H

#include <vector>
#include <iostream>
#include <map>
#include <algorithm>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "elevation_mapping/MyMap.h"
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace std;

namespace ConnectComponentLabeling
{
    class ConnectAeras
    {

private:

	double robotWide;

	vector< vector<float> > Map;

	int mapRows;

	int mapColumns;

	vector< vector<int> > TwoValueMapEdge;

	vector<int> parent; // Array of parent and child labels for connected area analysis

	vector< vector<int> > Label;

	vector< vector<int> > LabelEdge;

	vector< vector<int> > TwoValueMap;

	float Margin;

	float marginDistance;

	vector< vector<float> > labelObstacleSubmap; // Obstacle area of a certain label value

	float obstacleWeight;

	float obstacleLength;

	float obstacleHeight;

	float sizeOfObstacle;

	float densityOfObstacle;

	int numOfObstacle;

	vector<int> targetLabel;

	double resolution_;

	double angle;

	string gridmapFrameID;

	string robotFrameID;

	tf::TransformBroadcaster br;

	tf::TransformListener listener;

	vector<vector<double>> pathPosition;

	vector<vector<double>> rightWheelPath;

	vector<vector<double>> leftWheelPath;

	double robotInWorldX;

	double robotInWorldY;

	double robotInWorldZ;

	vector<vector<int>> indexOfObstacle;

	vector<vector<NewMap>> newMap;

	vector<ObstacleMap> ObstacleSubMap;

	vector< vector<NewMap> > subMap;

	float obstacleHeightLevel1;
	float obstacleHeightLevel2;
	float obstacleHeightLevel3;

	double obstacleWideLevel1;
	double obstacleWideLevel2;
	double obstacleWideLevel3;

	double obstacleLengthLevel1;
	double obstacleLengthLevel2;
	double obstacleLengthLevel3;

	double obstacleAngleLevel1;

	float motionHeight1; // Height of the vehicle body in wheeled mode

	float motionHeight2; // Height of the wheels in lifting mode

	float motionHeight3; // Height of the vehicle body in lifting mode

	double motionWheelDistance; // Wheel distance when lifting the vehicle body

	double marginDX;

	double marginDY;

	OBS obstaclePosition[5];

	double roverpositionX;
	double roverpositionY;
	double roverpositionZ;
	double roverAngleRow;
	double roverAnglePitch;
	double roverAngleYaw;

	double roverAndObstaclePositionInY;

	double roverAndObstacleDistance;
	int obstacleID;

public:
	ConnectAeras();

	bool IsObstacleInMap();

	double GetRoverAndObstacleDistance();

	void SetRobotParams(double robotwide, float motionheight1, float motionheight2, float motionheight3, double wheeldistance);

	void Set_Map(vector<vector<float>> Map_); //将elevation中读取的高度地图放到类的Map中

	void Set_Margin(float margin);

	void SetResolution(double resolution);

	int Search(vector<vector<int>> TwoValueMap_, int m, int n); //搜索像素点周围有效值个数的函数

	void Map_To_TwoValueMap(vector<vector<float>> Map_, float margin); 

	void TWO_PASS();

	int Label_Min(vector<int> labelNum);

	vector<int> Get_Label_Value(vector<vector<int>> Label_, int m, int n);

	vector<vector<int>> Get_ConnectMap();

	void mapEdge(vector<vector<int>> TwoValueMap_);

	void MoveValue(vector<int> &value);

	void DeletEdge(vector<vector<int>> Label_);

	vector<vector<float>> GetLabelObstacleSubmap();

	void GetCenterPositionInSubmapX(int label_, double &x);

	void GetCenterPositionInSubmapY(int label_, double &y);

	vector<int> GetAllLabelInMap();

	void SetAngle(double angle_);

	void SetObstacleData(OBS data[5]);

	void SetRoverAngle(double roverrow, double roverpitch, double roveryaw);

	double GetWeight();

	double Getlength();

	float GetHeight();

	//float GetDistanceX();

	vector<double> GetDistance();

	void ComputeDistanceY(int label_, double &distanceY);

	float GetDensity();

	int GetObstacleNum();

	bool GetTargetlabel();

	void ComputeIndex(vector<vector<int>> Label_);

	void SetDistanceMargin(float margin_, double marginx, double marginy);

	void ComputeNum();

	void ComputeWeight(int targetlabel_, double &weight);

	void ComputeLength(int targetlabel_, double &length);

	void ComputeDensity();

	void ComputeHeight(int targetlabel_, float &height);

	void ComputeDistanceX(int targetlabel_, double &distanceX);

	bool IndexInObstacleUp(int label_, int (&rowBegin)[2]);

	bool IndexInObstacleDown(int label_, int (&colBegin)[2]);

	bool IndexInObstacleLeft(int label_, int (&rowBegin)[2]);

	bool IndexInObstacleRight(int label_, int (&colBegin)[2]);

	//string GetObstacleType();

	void DetectObstacleType(int num);

	void SetRobotFrameID(string robotframe);

	void SetGridMapFrameID(string gridmapframe);

	void ReadRobotPath(vector<vector<double>> position);

	void SetRobotPosition(double x, double y);

	vector<vector<int>> GetObstacleCellIndex(int label_);

	vector<int> GetSubMapRowsAndColunms(int label_);

	void UpdateNewMap();

	void CellDataInWorld(int label_);

	void Obstacle();

	void ObstacleCenterPositionInWorld(int label_, int num);

	void ObstacleSubMapInworld(int label_, int num);

	int IndexMax(vector<int> index_);

	int IndexMin(vector<int> index_);

	void ObstacleDistanceWithPath(int label_, int num, double &x, double &y);

	void ObstacleLength(int label_, int num);

	void ObstacleWide(int label_, int num);

	vector<ObstacleMap> GetObstacleData();

	void ObstaclePlaneFitting(int label_, int num);

	bool gFittingPlane(vector<double> x, vector<double> y, vector<double> z, int n, double &a, double &b, double &c);

	void SetObstacleTypeData(float heightlevel1, float heightlevel2, float heightlevel3, double wideLevel1, double wideLevel2, double wideLevel3, double lengthLevel1, double lengthLevel2, double lengthLevel3, double anglelevel1);

	void PositionRelation(int num_);

	void OutputModeID(int num);

	void SetWheelPath();

	void ObstaclePositionRelation(int num);

	void ObstacleDistanceWithRobot(int label_, int num);

	void MinInVector(vector<double> arr, vector<int> label_, double &result, int &index_);

	void MinDistance(double arr[5], int &index_); 

	void MinDistance(vector<double> v, int &index_); 

	int IsAppearObstacle();

	void SetRoverPosition(double x, double y, double z);

	double DistanceNewX(double point, double child_point);

	double DistanceNewY(double point, double child_point);

	double DistanceNew(double pointx, double child_pointx, double pointy, double child_pointy);

	string GetModeID();

	string GetType();

	vector<ObstacleMap> FirstLayer();

	};

} // namespace ConnectAeras


#endif