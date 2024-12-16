/*
 * CheckObstacle.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// STL
#include <limits>
#include <algorithm>
#include <vector>

//pointcloud
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//pcl point
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

//elevation_mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

//file
#include<fstream>
#include<iomanip>

#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>

#include <math.h>

using namespace grid_map;
using namespace std;


ofstream out_txt_file;
double angleY = 0;

int Search(int arr[600][600], int m, int n, int row, int col)
{
	int a = 0;
	int num_left;
	int num_right;
	int num_up;
	int num_down;
	// If the input array element is at the beginning of each column, it has no upper value
	if (m == 0 && n != 0)
	{
		num_left = arr[m][n - 1];
		num_right = arr[m][n + 1];
		num_up = NULL;
		num_down = arr[m + 1][n];
		if (num_left == 0 && num_right == 0 && num_down == 0)
		{
			a = 0;
			return a;
		}
		// Pixels within the neighborhood have valid values
		else if (num_left != 0 || num_right != 0 || num_down != 0)
		{
			// Only 1 pixel is valid
			if (num_left + num_right + num_down == 1)
			{
				a = 1;
				return a;
			}
			// Only 2 pixels are valid
			else if (num_left + num_right + num_down == 2)
			{
				a = 2;
				return a;
			}
			// Only 3 pixels are valid
			else if (num_left + num_right + num_down == 3)
			{
				a = 3;
				return a;
			}

		}
	}
	// If the input array element is at the beginning of each column, it has no left value
	else if (m != 0 && n == 0)
	{
		num_left = NULL;
		num_right = arr[m][n + 1];
		num_up = arr[m - 1][n];
		num_down = arr[m + 1][n];
		if (num_right == 0 && num_up == 0 && num_down == 0)
		{
			a = 0;
			return a;
		}
		// Pixels within the neighborhood have valid values
		else if (num_right != 0 || num_up != 0 || num_down != 0)
		{
			// Only 1 pixel is valid
			if (num_right + num_up + num_down == 1)
			{
				a = 1;
				return a;
			}
			// Only 2 pixels are valid
			else if (num_right + num_up + num_down == 2)
			{
				a = 2;
				return a;
			}
			// Only 3 pixels are valid
			else if (num_right + num_up + num_down == 3)
			{
				a = 3;
				return a;
			}
		}
	}
	// If the input array element is at the beginning of each column, it has no upper and left value
	else if (m == 0 && n == 0)
	{
		num_left = NULL;
		num_right = arr[m][n + 1];
		num_up = NULL;
		num_down = arr[m + 1][n];
		if (num_right == 0 && num_down == 0)
		{
			a = 0;
			return a;
		}
		else if (num_right != 0 || num_down != 0)
		{
			if (num_right + num_down == 1)
			{
				a = 1;
				return a;
			}
			else if (num_right + num_down == 2)
			{
				a = 2;
				return a;
			}
		}
	}
	else if (m == row - 1 && n != col - 1)
	{
		num_left = arr[m][n - 1];
		num_right = arr[m][n + 1];
		num_up = arr[m - 1][n];
		num_down = NULL;
		if (num_left == 0 && num_right == 0 && num_up == 0)
		{
			a = 0;
			return a;
		}
		else if (num_left != 0 || num_right != 0 || num_up != 0)
		{
			if (num_left + num_right + num_up == 1)
			{
				a = 1;
				return a;
			}
			else if (num_left + num_right + num_up == 2)
			{
				a = 2;
				return a;
			}
			else if (num_left + num_right + num_up == 3)
			{
				a = 3;
				return a;
			}

		}
	}
	else if (m != row - 1 && n == col - 1)
	{
		num_left = arr[m][n - 1];
		num_right = NULL;
		num_up = arr[m - 1][n];
		num_down = arr[m + 1][n];
		if (num_left == 0 && num_up == 0 && num_down == 0)
		{
			a = 0;
			return a;
		}
		else if (num_left != 0 || num_up != 0 || num_down != 0)
		{
			if (num_left + num_up + num_down == 1)
			{
				a = 1;
				return a;
			}
			else if (num_left + num_up + num_down == 2)
			{
				a = 2;
				return a;
			}
			else if (num_left + num_up + num_down == 3)
			{
				a = 3;
				return a;
			}

		}

	}
	else if (m == row - 1 && n == col - 1)
	{
		num_left = arr[m][n - 1];
		num_right = NULL;
		num_up = arr[m - 1][n];
		num_down = NULL;
		if (num_left == 0 && num_up == 0)
		{
			a = 0;
			return a;
		}
		else if (num_left != 0 || num_up != 0)
		{
			if (num_left + num_up == 1)
			{
				a = 1;
				return a;
			}
			else if (num_left + num_up == 2)
			{
				a = 2;
				return a;
			}

		}
	}
	else
	{
		num_left = arr[m][n - 1];
		num_right = arr[m][n + 1];
		num_up = arr[m - 1][n];
		num_down = arr[m + 1][n];

		if (num_left == 0 && num_right == 0 && num_up == 0 && num_down == 0)
		{
			a = 0;
			return a;
		}
		else if (num_left != 0 || num_right != 0 || num_up != 0 || num_down != 0)
		{
			if (num_left + num_right + num_up + num_down == 1)
			{
				a = 1;
				return a;
			}
			else if (num_left + num_right + num_up + num_down == 2)
			{
				a = 2;
				return a;
			}
			else if (num_left + num_right + num_up + num_down == 3)
			{
				a = 3;
				return a;
			}
			else if (num_left + num_right + num_up + num_down == 4)
			{
				a = 4;
				return a;
			}

		}
	}
	return a;
}


int* Get_Label_Value(int label_value[600][600], int m, int n, int row, int col )
{
	// Function to get the label values within the neighborhood

	/* Used to store the pixel values of the neighborhood. 
	 Positions 0-3 represent the values within the neighborhood, 
	 position 4 represents the number of non-zero values */
	int* value = new int[5];
	for (int i = 0; i < 5; i++)
	{
		value[i] = 0;
	}
	if (m == 0 && n == 0)
	{
		value[0] = label_value[m + 1][n];
		value[1] = label_value[m][n + 1];
		if (value[0] == 0 && value[1] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0)
		{
			value[4] = 1;
			return value;
		}
		else
		{
			value[4] = 2;
			return value;
		}

	}
	else if (m == row - 1 && n == col - 1)
	{
		value[0] = label_value[m - 1][n];
		value[1] = label_value[m][n - 1];
		if (value[0] == 0 && value[1] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0)
		{
			value[4] = 1;
			return value;
		}
		else
		{
			value[4] = 2;
			return value;
		}

	}
	else if (m == 0 && n == col - 1)
	{
		value[0] = label_value[m + 1][n];
		value[1] = label_value[m][n - 1];
		if (value[0] == 0 && value[1] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0)
		{
			value[4] = 1;
			return value;
		}
		else
		{
			value[4] = 2;
			return value;
		}
	}
	else if (m == row - 1 && n == 0)
	{
		value[0] = label_value[m][n + 1];
		value[1] = label_value[m - 1][n];
		if (value[0] == 0 && value[1] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0)
		{
			value[4] = 1;
			return value;
		}
		else
		{
			value[4] = 2;
			return value;
		}
	}
	else if (m == 0 && n != 0)
	{
		value[0] = label_value[m][n + 1];
		value[1] = label_value[m + 1][n];
		value[2] = label_value[m][n - 1];
		if (value[0] == 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0)
		{
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0)
		{
			value[1] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 2;
			return value;
		}
		else
		{
			value[4] = 3;
			return value;
		}

	}
	else if (m != 0 && n == 0)
	{
		value[0] = label_value[m][n + 1];
		value[1] = label_value[m + 1][n];
		value[2] = label_value[m - 1][n];
		if (value[0] == 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0)
		{
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0)
		{
			value[1] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 2;
			return value;
		}
		else
		{
			value[4] = 3;
			return value;
		}

	}
	else if (m == row - 1 && n != col - 1)
	{
		value[0] = label_value[m][n + 1];
		value[1] = label_value[m][n - 1];
		value[2] = label_value[m - 1][n];
		if (value[0] == 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0)
		{
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0)
		{
			value[1] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 2;
			return value;
		}
		else
		{
			value[4] = 3;
			return value;
		}
	}
	else if (m != row - 1 && n == col - 1)
	{
		value[0] = label_value[m][n - 1];
		value[1] = label_value[m + 1][n];
		value[2] = label_value[m - 1][n];
		if (value[0] == 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0)
		{
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0)
		{
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0)
		{
			value[1] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0)
		{
			value[0] = value[2];
			value[4] = 2;
			return value;
		}
		else
		{
			value[4] = 3;
			return value;
		}
	}
	else
	{
		value[0] = label_value[m][n - 1];
		value[1] = label_value[m + 1][n];
		value[2] = label_value[m - 1][n];
		value[3] = label_value[m][n - 1];
		if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0)
		{
			value[4] = 0;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0 && value[3] == 0)
		{
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0 && value[3] == 0)
		{
			value[0] = value[1];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0 && value[3] == 0)
		{
			value[0] = value[2];
			value[4] = 1;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] != 0)
		{
			value[0] = value[3];
			value[4] = 1;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0 && value[3] == 0)
		{
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0 && value[3] == 0)
		{
			value[1] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] == 0 && value[3] != 0)
		{
			value[1] = value[3];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0 && value[3] == 0)
		{
			value[0] = value[2];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] == 0 && value[3] != 0)
		{
			value[0] = value[3];
			value[4] = 2;
			return value;
		}
		else if (value[0] == 0 && value[1] == 0 && value[2] != 0 && value[3] != 0)
		{
			value[0] = value[2];
			value[1] = value[3];
			value[4] = 2;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] != 0 && value[3] == 0)
		{
			value[4] = 3;
			return value;
		}
		else if (value[0] != 0 && value[1] != 0 && value[2] == 0 && value[3] != 0)
		{
			value[2] = value[3];
			value[4] = 3;
			return value;
		}
		else if (value[0] != 0 && value[1] == 0 && value[2] != 0 && value[3] != 0)
		{
			value[1] = value[3];
			value[4] = 3;
			return value;
		}
		else if (value[0] == 0 && value[1] != 0 && value[2] != 0 && value[3] != 0)
		{
			value[0] = value[3];
			value[4] = 3;
			return value;
		}
		else
		{
			value[4] = 4;
			return value;
		}
	}

}

int Label_Min(int label_get[600][600], int m, int n, int row, int col )
{
	// Function to get the minimum label values within the neighborhood

	int Min_Num(int num1, int num2);
	int* label_num;
	label_num = Get_Label_Value(label_get, m, n, row, col);
	switch (label_num[4])
	{
	case 1:
		return label_num[0];
	case 2:
		return Min_Num(label_num[0], label_num[1]);
		break;

	case 3:
		return Min_Num(label_num[0], Min_Num(label_num[1], label_num[2]));
		break;

	case 4:
		return Min_Num(label_num[0], Min_Num(label_num[1], Min_Num(label_num[2], label_num[3])));
		break;
	}
	
}

int Min_Num(int num1, int num2)
{
	if (num1 >= num2)
	{
		return num2;
	}
	else
	{
		return num1;
	}
}

int Get_nums(int LP[], int size)
{
	int num = 0;
	for (int i = 0; i < size; i++)
	{
		if (LP[i] < 0)
		{
			num += 1;
		}
	}
	return num;
}

// Get the vertical size of the obstacle
int Get_weight(int arr[600][600], int a)
{
	int num = 0;
	bool is = false;
	for (int i = 0; i < 600; i++)
	{
		for (int j = 0; j < 600; j++)
		{
			if (arr[i][j] == a)
			{
				is = true;
				continue;
			}
		}
		if (is)
		{
			num += 1;
			is = false;
		}
	}
	return num;
}

// Get the horizontal size of the obstacle
int Get_length(int arr[600][600], int a)
{
	int num = 0;
	int temp = 0;
	for (int i = 0; i < 600; i++)
	{
		for (int j = 0; j < 600; j++)
		{
			if (arr[i][j] == a)
			{
				num += 1;
			}
		}
		if (temp <= num)
		{
			temp = num;
			num = 0;
		}
		else
		{
			num = 0;
			continue;
		}
	}

	return temp;
}


double GetDistanceYObstacleAndRobot(int arr[600][600], int a)
{
	double d = 0;
	int length = Get_length(arr, a);
	int weight = Get_weight(arr, a);
	int init_j = 0;
	

	for (int i = 0; i < 600; i++)
	{
		for (int j = 0; j < 600; j++)
		{
			if(arr[i][j] == a)
			{
				init_j = j;
				break;
			}
		}
	}
	d = 30 - ((length / 2) + init_j);
	return d;
}

double GetDistanceXObstacleAndRobot(int arr[600][600], int a)
{
	double d = 0;
	int length = Get_length(arr, a);
	int weight = Get_weight(arr, a);
	int init_i = 0;
	

	for (int j = 0; j < 600; j++)
	{
		for (int i = 0; i < 600; i++)
		{
			if(arr[i][j] == a)
			{
				init_i = i;
				break;
			}
		}
	}
	d = ((weight / 2) + init_i)-30;
	return d;
}


void GetPoseAngleCallback(const nav_msgs::Odometry &odom)
{
	tf::Quaternion quat;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	angleY = yaw;
}

 void TestCallback(const grid_map_msgs::GridMap& message)
 {
	
	
	int parent[999999] = { 0 };
	grid_map::GridMap LoadMap;
    GridMapRosConverter::fromMessage(message,LoadMap);

    double lengthX = LoadMap.getLength().x();
    double lengthY = LoadMap.getLength().y();
    double resolution = LoadMap.getResolution();

    grid_map::Position XY_offream = LoadMap.getPosition();
	const grid_map::Position& initPosition = LoadMap.getPosition();

	// 0.3 represents the margin, the value of the margin can be obtained from the parameter server
    const int lengthInXSubmapI = static_cast<int>(lengthX / resolution + 2 * 0.3);
    const int lengthInYSubmapI = static_cast<int>( lengthY/ resolution + 2 *0.3);

    int Length_in_obstacle[lengthInYSubmapI];
    int wide_in_obstacle[lengthInXSubmapI];

	// Submap iterator initialization
	const grid_map::Position topLeftPosition(initPosition(0) + lengthX / 2, initPosition(1) + lengthY / 2);
	grid_map::Index submapTopLeftIndex;
	LoadMap.getIndex(topLeftPosition, submapTopLeftIndex);
	const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);
	grid_map::Matrix& elevationData = LoadMap["elevation"];

    float margin_data = 0.04; // Threshold

	// Define an array with the size of the current map rectangle to store the height data of each grid in the map
    float diff_height[lengthInXSubmapI * lengthInYSubmapI]; 

    int two_map_data[600][600];
	float map[600][600] = {0};
	float two_data;
	int array[lengthInXSubmapI * lengthInYSubmapI] = {0};
   
	int LB = 0;
	int num = 0;
	int *temp;
	int biaoqian = 0;
	int label[600][600] = {0};

	cout << LoadMap.getSize() << endl;

	for (grid_map::SubmapIterator iterator(LoadMap, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
	two_data = elevationData(index(0), index(1));
	grid_map::Index size_In_map = iterator.getSubmapIndex();
	
	    if (!isnan(two_data))
        {

            map[size_In_map(0)][size_In_map(1)] = two_data;
			if(two_data > margin_data)
			{
				two_map_data[size_In_map(0)][size_In_map(1)] = 1;
			}
			else{
				two_map_data[size_In_map(0)][size_In_map(1)] = 0;
			}
        }
        else{
            two_map_data[size_In_map(0)][size_In_map(1)] = 0;
			// array[i] = 0;
        }
	 }

	for (int iq = 0; iq < 600; iq++)
	{
		for (int  jq= 0; jq < 600; jq++)
		{
			if (two_map_data[iq][jq] )
			{
				// Get the number of valid values around the pixel
				temp = Get_Label_Value(label, iq, jq, 600, 600);
				num = Search(two_map_data, iq, jq, 600, 600);
				// If there are no valid values around, give it a root label
				if (!num)
				{
					LB += 1;
					label[iq][jq] = LB;
					biaoqian = label[iq][jq];
					parent[biaoqian] = -1;
				}
				else if (num && temp[4] == 0)
				{
					LB += 1;
					label[iq][jq] = LB;
					biaoqian = label[iq][jq];
					parent[biaoqian] = -1;
				}
				else if (num && temp[4] ==1)
				{
					label[iq][jq] = Label_Min(label, iq, jq, 600, 600);
				}
				else if (num && temp[4] == 2)
				{
					label[iq][jq] = Label_Min(label, iq, jq, 600, 600);
					biaoqian = label[iq][jq];
					if (temp[0] >= temp[1])
					{
						parent[temp[0]] = temp[1];
					}
					else
					{
						parent[temp[1]] = temp[0];
					}
				}
				else if (num && temp[4] == 3)
				{
					label[iq][jq] = Label_Min(label, iq, jq, 600, 600);
					biaoqian = label[iq][jq];
					if (temp[2] == biaoqian)
					{
						parent[temp[0]] = biaoqian;
						parent[temp[1]] = biaoqian;
					}
					else if (temp[1] == biaoqian)
					{
						parent[temp[0]] = biaoqian;
						parent[temp[2]] = biaoqian;
					}
					else if (temp[0] == biaoqian)
					{
						parent[temp[1]] = biaoqian;
						parent[temp[2]] = biaoqian;
					}
				}
				else if (num && temp[4] == 4)
				{
					label[iq][jq] = Label_Min(label, iq, jq, 600, 600);
					biaoqian = label[iq][jq];
					if (temp[0] == biaoqian)
					{
						parent[temp[1]] = biaoqian;
						parent[temp[2]] = biaoqian;
						parent[temp[3]] = biaoqian;
					}
					else if (temp[1] == biaoqian)
					{
						parent[temp[0]] = biaoqian;
						parent[temp[2]] = biaoqian;
						parent[temp[3]] = biaoqian;
					}
					else if (temp[2] == biaoqian)
					{
						parent[temp[0]] = biaoqian;
						parent[temp[1]] = biaoqian;
						parent[temp[3]] = biaoqian;
					}
					else if (temp[3] == biaoqian)
					{
						parent[temp[0]] = biaoqian;
						parent[temp[1]] = biaoqian;
						parent[temp[2]] = biaoqian;
					}
				}
			}
			else
			{
				continue;
			}
		}

		/*TWO - PASS*/
		for (int g = 0; g < 600; g++)
		{
			for (int z = 0; z < 600; z++)
			{
				if (label[g][z])
				{
					if (parent[label[g][z]] != -1)
					{
						label[g][z] = parent[label[g][z]];
					}
					else
					{
						continue;
					}
				}
			}
		}

	}


	int obstacle_length = 0;
	int obstacle_weight = 0;
	double distanceY = 0;
	double distanceX = 0;

	int obstacle_nums = Get_nums(parent, 360000);

	Eigen::Vector2d testPosition = LoadMap.getPosition();
	cout << "getposition()输出的值：" << endl
		 << testPosition(0) << endl
		 << testPosition(1) << endl;
	Position3 cellTestPosition;

	
	Index cellTestIndex;
	cellTestIndex(0) = 10;
	cellTestIndex(1) = 10;
	LoadMap.getPosition3("elevation", cellTestIndex,cellTestPosition);
	cout << "Index=（1，1）处的position值为：" << endl
		 << cellTestPosition << endl;

	/****************************************Write to file***************************************************/
	out_txt_file.open("/home/jie/slam_project/map04_ws/src/data/WMRpro.txt", ios::out | ios::app);
	out_txt_file << fixed;
	out_txt_file << "Number of obstacles"
				<< "\t"
				<< "Vertical size of obstacle"
				<< "\t"
				<< "Horizontal size of obstacle" << endl;
	if(obstacle_nums)
	{
		cout << "Number of obstacles:" << endl;
		cout << zhangaiwu_nums << endl;
		for (int ji = 0; ji < zhangaiwu_nums; ji++)
		{
			obstacle_length = Get_length(label, ji + 1);
			obstacle_weight = Get_weight(label, ji + 1);
			distanceY = GetDistanceYObstacleAndRobot(label, ji + 1);
			distanceX = -GetDistanceXObstacleAndRobot(label, ji + 1);

			double Y = (distanceY * resolution) * cos(angleY) - (distanceX * resolution) * sin(angleY);
			double X = (distanceY * resolution) * sin(angleY) + (distanceX * resolution) * cos(angleY);

			cout << "Length of obstacle " << ji + 1 << ":" << endl;
			cout << obstacle_length * resolution << endl;
			cout << "Width of obstacle " << ji + 1 << ":" << endl;
			cout << obstacle_weight * resolution << endl;
			cout << "Y distance of obstacle " << ji + 1 << ":" << endl;
			cout << Y << endl;
			cout << "X distance of obstacle " << ji + 1 << ":" << endl;
			cout << X << endl;
			out_txt_file << zhangaiwu_nums << "\t"
						<< "\t" << obstacle_length * resolution << "\t"
						<< obstacle_weight * resolution << endl;
		}
	}
	out_txt_file.close();
 }


 int main(int argc, char **argv)
 {
	grid_map::GridMap LoadMap;
	ros::init(argc, argv, "CheckObstacle");
	ros::NodeHandle n;
	ros::Subscriber angle;
	ros::Subscriber TwoValuemap;
	
	std::string poseTopic = "/odom";
	TwoValuemap = n.subscribe("/elevation_mapping/elevation_map", 10, &TestCallback);
	angle = n.subscribe(poseTopic, 10, &GetPoseAngleCallback);

	ros::spin();     
	return 0;
 }

 