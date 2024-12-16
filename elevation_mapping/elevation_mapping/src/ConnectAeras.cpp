/*
 * ConnectAeras.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include <cmath>
#include "ros/ros.h"
#include "elevation_mapping/ConnectAeras.h"

using namespace std;

namespace ConnectComponentLabeling
{
    /*************************Two-Pass Connect Component labeling****************************/
    bool ConnectAeras::IsObstacleInMap()
    {
        if (parent.size() == 1)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void ConnectAeras::SetRobotParams(double robotwide, float motionheight1, float motionheight2, float motionheight3, double wheeldistance)
    {
        robotWide = robotwide;
        motionHeight1 = motionheight1;
        motionHeight2 = motionheight2;
        motionHeight3 = motionheight3;
        motionWheelDistance = wheeldistance;
    }

    void ConnectAeras::SetResolution(double resolution)
    {
        resolution_ = resolution;
    }

    ConnectAeras::ConnectAeras()
    {
        Margin = 0;
    }

    void ConnectAeras::Set_Margin(float margin)
    {
        // Set the height threshold for calculating the binary map
        Margin = margin;
    }

    void ConnectAeras::Set_Map(vector<vector<float>> Map_)
    {
        // Read the original height map
        for (int i = 0; i < Map_.size(); i++)
        {
            Map.push_back(Map_[i]);
        }
        mapRows = Map_.size();
        mapColumns = Map_[0].size();
    }

    int ConnectAeras::Search(vector<vector<int>> TwoValueMap_, int m, int n)
    {
        // Check if there are valid values in the 8-neighborhood cells of a cell in the binary map
        int cellLeft = TwoValueMap_[m][n - 1];
        int cellRight = TwoValueMap_[m][n + 1];
        int cellUp = TwoValueMap_[m - 1][n];
        int cellDown = TwoValueMap_[m + 1][n];
        int cellLeftAndUp = TwoValueMap_[m - 1][n - 1];
        int cellRightAndUp = TwoValueMap_[m - 1][n + 1];
        int cellLeftAndDwon = TwoValueMap_[m + 1][n - 1];
        int cellRightAndDown = TwoValueMap_[m + 1][n + 1];
        int a = cellDown + cellLeft + cellLeftAndDwon + cellLeftAndUp + cellRight + cellRightAndDown + cellRightAndUp + cellUp;

        return a;
    }

    void ConnectAeras::Map_To_TwoValueMap(vector<vector<float>> Map_, float margin)
    {
        // Convert the original height map to a binary map
        for (int i = 0; i < Map_.size(); i++)
        {
            vector<int> raw;
            for (int j = 0; j < Map_[i].size(); j++)
            {
                if (fabs(Map_[i][j]) > margin)
                {
                    raw.push_back(1);
                }
                else
                {
                    raw.push_back(0);
                }
            }
            TwoValueMap.push_back(raw);
        }
    }

    void ConnectAeras::TWO_PASS() 
    {
        // Connected area analysis

        int valueOnTree = 0;
        int ableTwoValue = 0;
        int temporaryLabel = 0;
        int temp = -1;

        //Set union-find tree
        parent.assign(1, 0);


        Map_To_TwoValueMap(Map, Margin);
        mapEdge(TwoValueMap);

        // Set the label matrix with edges
        for (int k = 0; k < TwoValueMapEdge.size(); k++)
        {
            vector<int> hub;
            for (int g = 0; g < TwoValueMapEdge[k].size(); g++)
            {
                hub.push_back(0);
            }
            LabelEdge.push_back(hub);
        }

        for (int i = 1; i < TwoValueMapEdge.size() - 1; i++)
        {

            for (int j = 1; j < TwoValueMapEdge[i].size() - 1; j++)
            {
                if (TwoValueMapEdge[i][j] == 1)
                {
                    vector<int> neighborLabelValue(Get_Label_Value(LabelEdge, i, j));
                    //ROS_INFO("neighborLabelValue OK");
                    ableTwoValue = Search(TwoValueMapEdge, i, j);
                    //ROS_INFO("Search OK");
                    if (!ableTwoValue)
                    {
                        temporaryLabel += 1;
                        LabelEdge[i][j] = temporaryLabel;
                        valueOnTree = LabelEdge[i][j];
                        parent.push_back(-1);
                    }
                    else if (ableTwoValue && neighborLabelValue[8] == 0)
                    {
                        temporaryLabel += 1;
                        LabelEdge[i][j] = temporaryLabel;
                        valueOnTree = LabelEdge[i][j];
                        parent.push_back(-1);
                    }
                    else if (ableTwoValue && neighborLabelValue[8] != 0)
                    {
                        if (neighborLabelValue[8] == 1)
                        {
                            LabelEdge[i][j] = Label_Min(neighborLabelValue);
                            valueOnTree = LabelEdge[i][j];
                        }
                        else
                        {
                            LabelEdge[i][j] = Label_Min(neighborLabelValue);
                            valueOnTree = LabelEdge[i][j];
                            for (int it = 0; it < neighborLabelValue[8]; it++)
                            {
                                if (neighborLabelValue[it] == valueOnTree)
                                {
                                    for (int is = 0; is < neighborLabelValue[8]; is++)
                                    {
                                        if (is == it || neighborLabelValue[is] == valueOnTree)
                                        {
                                            continue;
                                        }
                                        else
                                        {
                                            parent[neighborLabelValue[is]] = valueOnTree;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
        }

        for (int o = 1; o < LabelEdge.size() - 1; o++)
        {
            for (int z = 1; z < LabelEdge[o].size() - 1; z++)
            {
                if (LabelEdge[o][z])
                {
                    while (parent[LabelEdge[o][z]] != -1)
                    {
                        LabelEdge[o][z] = parent[LabelEdge[o][z]];
                    }
                }
            }
        }

        DeletEdge(LabelEdge);
    }

    // Get the label values around the target cell
    vector<int> ConnectAeras::Get_Label_Value(vector<vector<int>> Label_, int m, int n)
    {
        vector<int> valueOnNeighbor(9, 0);
        valueOnNeighbor[0] = Label_[m][n - 1];
        valueOnNeighbor[1] = Label_[m][n + 1];
        valueOnNeighbor[2] = Label_[m - 1][n];
        valueOnNeighbor[3] = Label_[m + 1][n];
        valueOnNeighbor[4] = Label_[m - 1][n - 1];
        valueOnNeighbor[5] = Label_[m - 1][n + 1];
        valueOnNeighbor[6] = Label_[m + 1][n - 1];
        valueOnNeighbor[7] = Label_[m + 1][n + 1];
        valueOnNeighbor[8] = 0;

        vector<int> isEableValue(8, 0);
        for (int i = 0; i < 8; i++)
        {
            if (valueOnNeighbor[i] != 0)
            {
                isEableValue[i] = 1;
            }
            valueOnNeighbor[8] += isEableValue[i];
        }

        switch (valueOnNeighbor[8])
        {
        case 0:
            return valueOnNeighbor;

        case 1:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 2:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 3:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 4:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 5:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 6:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 7:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        case 8:
            MoveValue(valueOnNeighbor);
            return valueOnNeighbor;
            break;

        default:
            break;
        }
    }

    int ConnectAeras::Label_Min(vector<int> labelNum)
    {
        switch (labelNum[8])
        {
        case 1:
            return labelNum[0];
            break;

        case 2:
            return min(labelNum[0], labelNum[1]);
            break;

        case 3:
            return min(labelNum[0], min(labelNum[1], labelNum[2]));
            break;

        case 4:
            return min(labelNum[0], min(labelNum[1], min(labelNum[2], labelNum[3])));
            break;

        case 5:
            return min(labelNum[0], min(labelNum[1], min(labelNum[2], min(labelNum[3], labelNum[4]))));
            break;

        case 6:
            return min(labelNum[0], min(labelNum[1], min(labelNum[2], min(labelNum[3], min(labelNum[4], labelNum[5])))));
            break;

        case 7:
            return min(labelNum[0], min(labelNum[1], min(labelNum[2], min(labelNum[3], min(labelNum[4], min(labelNum[5], labelNum[6]))))));
            break;

        case 8:
            return min(labelNum[0], min(labelNum[1], min(labelNum[2], min(labelNum[3], min(labelNum[4], min(labelNum[5], min(labelNum[6], labelNum[7])))))));
            break;

        default:
            break;
        }
    }

    vector<vector<int>> ConnectAeras::Get_ConnectMap()
    {
        return Label;
    }

    void ConnectAeras::mapEdge(vector<vector<int>> TwoValueMap_)
    {
        for (int i = 0; i < TwoValueMap_.size() + 2; i++)
        {
            vector<int> raw;
            if (i == 0 || i == TwoValueMap_.size() + 1)
            {
                raw.assign(TwoValueMap_[0].size() + 2, 0);
            }
            else
            {
                raw.push_back(0);
                raw.insert(raw.begin() + 1, TwoValueMap_[i - 1].begin(), TwoValueMap_[i - 1].end());
                raw.push_back(0);
            }
            TwoValueMapEdge.push_back(raw);
        }
    }

    // Sort non-zero elements to the left side of the vector and zero elements to the right side of the vector.
    void ConnectAeras::MoveValue(vector<int> &value)
    {
        int low, hight;
        for (low = 0, hight = low; low < value.size() - 1; low++)
        {
            if (value[low] != 0)
            {
                value[hight] = value[low];
                hight++;
            }
        }
        for (; hight < value.size() - 1; hight++)
        {
            value[hight] = 0;
        }
    }

    void ConnectAeras::DeletEdge(vector<vector<int>> Label_)
    {
        for (int i = 1; i < Label_.size() - 1; i++)
        {
            vector<int> tempLabel;
            for (int j = 1; j < Label_[i].size() - 1; j++)
            {
                tempLabel.push_back(Label_[i][j]);
            }
            Label.push_back(tempLabel);
        }
    }

    /******************************Control Index****************************************/

    //Get all label value of the label map.
    vector<int> ConnectAeras::GetAllLabelInMap()
    {
        vector<int> allLabel;
        if (!parent.empty())
        {
            for (int i = 0; i < parent.size(); i++)
            {
                if (parent[i] == -1)
                {
                    allLabel.push_back(i);
                }
            }
        }
        else
        {
            cout << "Vector parent is empty?" << endl;
        }

        return allLabel;
    }

    //Set robot yaw angle. (no)
    void ConnectAeras::SetAngle(double angle_)
    {
        angle = angle_;
    }

    //Get the row begin numbers of sub map with target label value .
    bool ConnectAeras::IndexInObstacleUp(int label_, int (&rowBegin)[2])
    {
        if (IsObstacleInMap())
        {
            for (int i = 0; i < Label.size(); i++)
            {
                for (int j = 0; j < Label[i].size(); j++)
                {
                    if (Label[i][j] == label_)
                    {
                        rowBegin[0] = i;
                        rowBegin[1] = j;
                        return true;
                    }
                }
            }
        }
        else
        {
            cout << "Without obstacle sub map?" << endl;
            return false;
        }
    }

    //Get the col begin numbers of sub map with target label value.
    bool ConnectAeras::IndexInObstacleDown(int label_, int (&colBegin)[2])
    {
        if (IsObstacleInMap())
        {
            for (int i = Label.size() - 1; i >= 0; i--)
            {
                for (int j = 0; j < Label[i].size(); j++)
                {
                    if (Label[i][j] == label_)
                    {
                        colBegin[0] = i;
                        colBegin[1] = j;
                        return true;
                    }
                }
            }
        }
        else
        {
            cout << "Without obstacle sub map?" << endl;
            return false;
        }
    }

    bool ConnectAeras::IndexInObstacleLeft(int label_, int (&rowBegin)[2])
    {
        if (IsObstacleInMap())
        {
            for (int j = 0; j < Label[0].size(); j++)
            {
                for (int i = 0; i < Label.size(); i++)
                {
                    if (Label[i][j] == label_)
                    {
                        rowBegin[0] = i;
                        rowBegin[1] = j;
                        return true;
                    }
                }
            }
        }
        else
        {
            cout << "Without obstacle sub map?" << endl;
            return false;
        }
    }

    bool ConnectAeras::IndexInObstacleRight(int label_, int (&colBegin)[2])
    {
        if (IsObstacleInMap())
        {
            for (int j = Label[0].size() - 1; j >= 0; j--)
            {
                for (int i = 0; i < Label.size(); i++)
                {
                    if (Label[i][j] == label_)
                    {
                        colBegin[0] = i;
                        colBegin[1] = j;
                        return true;
                    }
                }
            }
        }
        else
        {
            cout << "Without obstacle sub map?" << endl;
            return false;
        }
    }

    //Get the center position x of sub map with target label value.
    void ConnectAeras::GetCenterPositionInSubmapX(int label_, double &x)
    {
        double length;
        int rowBegin[2];
        ComputeLength(label_, length);
        IndexInObstacleUp(label_, rowBegin);
        x = ((Label.size() * 0.5) - rowBegin[0]) * resolution_ - (length * 0.5);
    }

    //Get the center position y of sub map with target label value.
    void ConnectAeras::GetCenterPositionInSubmapY(int label_, double &y)
    {
        double weight;
        int colBegin[2];
        ComputeWeight(label_, weight);
        IndexInObstacleLeft(label_, colBegin);
        y = ((Label[0].size() * 0.5) - colBegin[1]) * resolution_ - (weight * 0.5);
    }

    //output the obstacle sub map with target label value.
    vector<vector<float>> ConnectAeras::GetLabelObstacleSubmap()
    {
        if (GetTargetlabel())
        {
            //FindLabelObstacleSubmap(targetLabel);
            return labelObstacleSubmap;
        }
        else
        {
            cout << "Please set margin of distance or without obstacle in map." << endl;
        }
    }


    //Get length of the obstacle.
    double ConnectAeras::Getlength()
    {
        if (!parent.empty())
        {
            double length;
            ComputeLength(1, length);
            obstacleLength = length;
            return obstacleLength;
        }
        else
        {
            float tmp = nan("");
            cout << "Without obstacle in map." << endl;
            return tmp;
        }
    }

    //Get wide of the obstacle.
    double ConnectAeras::GetWeight()
    {
        if (!parent.empty())
        {
            double weight;
            ComputeWeight(1, weight);
            obstacleWeight = weight;
            return obstacleWeight;
        }
        else
        {
            float tmp = nan("");
            cout << "Without obstacle in map." << endl;
            return tmp;
        }
    }

    //Get obstacle's density in the map.(no)
    float ConnectAeras::GetDensity()
    {
        if (!parent.empty())
        {
            ComputeDensity();
            return densityOfObstacle;
        }
        else
        {
            float tmp = nan("");
            cout << "Without obstacle in map." << endl;
            return tmp;
        }
    }

    //Get obstacle's number in the map.(no)
    int ConnectAeras::GetObstacleNum()
    {
        ComputeNum();
        return numOfObstacle;
    }


    void ConnectAeras::MinInVector(vector<double> arr,vector<int> labelTmp, double &result, int &index_)
    {
        double tmp;
        double lenTmp = 0.0, wideTmp = 0.0;
        double S = 0.0;
         double STmp;
        if (!arr.empty() && !labelTmp.empty())
        {
            result = arr[0];
            for (int i = 0; i < arr.size(); i++)
            {
                ComputeLength(labelTmp[i],lenTmp);
                ComputeWeight(labelTmp[i],wideTmp);
                STmp = lenTmp * wideTmp; 
                if (result > arr[i] && S < STmp)
                {
                    result = arr[i];
                    S = STmp;
                    index_ = i;
                }
                else
                {
                    continue;
                }
            }
        }
        else
        {
            result = nan(" ");
            index_ = nan(" ");
        }
    }


    //Get the obstacle's label value that satisfy the condition
    bool ConnectAeras::GetTargetlabel() 
    { 
        targetLabel.clear();
        if (IsObstacleInMap())
        {
            vector<int> label_;
            double D;
            int tmp = 0;
            vector<double> dis;
            double output=0;
            int min_index=0;

            if (!GetAllLabelInMap().empty())
            {
                label_ = GetAllLabelInMap();
                // ROS_INFO("label_ num is: %d", label_.size());
                for (int i = 0; i < label_.size(); i++)
                {
                    ComputeDistanceX(label_[i], D);
                    
                    if (D > 0)
                    {
                        targetLabel.push_back(label_[i]);
                        dis.push_back(D);
                        // ROS_INFO("targetlabel num is: %d", targetLabel.size());
                    }
                    else
                    {
                        ROS_INFO("The D is :%.4f",D);
                        ROS_ERROR("The Distance X is < 0");
                    }
                }
                MinInVector(dis, label_,output, min_index);
                if (!isnan(output) &&!isnan(min_index))
                {
                    double wideMargin = 0.0;
                    double lengthMargin = 0.0;
                    float heightMargin = 0.0;
                    ComputeWeight(label_[min_index],wideMargin);
                    ComputeLength(label_[min_index],lengthMargin);
                    ComputeHeight(label_[min_index],heightMargin);
                    if (wideMargin > resolution_ && lengthMargin > resolution_)
                    {
                            targetLabel.push_back(label_[min_index]);
                            return true;
                    }
                    else
                    {
                        return false;
                    }
                
                }
                else
                {
                    return false;
                }
                if (!targetLabel.empty())
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                ROS_INFO("Without obstacle in the map");
                return false;
            }
        }
        else
        {
            ROS_INFO("Without obstacle in the map");
            return false;
        }
    }

    //Get the value of distance between target obstacle and robot.
    vector<double> ConnectAeras::GetDistance()
    {
        double tmpX, tmpY;
        vector<double> Distance;
        if (GetTargetlabel())
        {
            ComputeDistanceX(1, tmpX);
            ComputeDistanceY(1, tmpY);
            Distance.push_back(tmpX);
            Distance.push_back(tmpY);
        }
        else
        {
            tmpX = nan("");
            tmpY = nan("");
            cout << "Without distance." << endl;
            Distance.push_back(tmpX);
            Distance.push_back(tmpY);
        }
        return Distance;
    }

    //Compute the distance of Y between obstacle and robot.
    void ConnectAeras::ComputeDistanceY(int label_, double &distanceY)
    {
        double x, y;
        GetCenterPositionInSubmapX(label_, x);
        GetCenterPositionInSubmapY(label_, y);
        float centerXInMap, centerYInMap;
        centerXInMap = (Map.size() * resolution_) / 2;
        centerYInMap = (Map[0].size() * resolution_) / 2;
        distanceY = y * cos(angle) - x * sin(angle);
    }

    //Set the margin of distance between obstacle and robot.
    void ConnectAeras::SetDistanceMargin(float margin_, double marginx, double marginy)
    {
        marginDistance = margin_;
        marginDX = marginx;
        marginDY = marginy;
    }

    //Compute the number of obstacle in the map.(no)
    void ConnectAeras::ComputeNum()
    {
        for (int i = 1; i < parent.size(); i++)
        {
            if (parent[i] == -1)
            {
                numOfObstacle += 1;
            }
        }
    }

    void ConnectAeras::ComputeWeight(int targetlabel_, double &weight)
    {
        int left[2], right[2];
        if (IndexInObstacleLeft(targetlabel_, left) && IndexInObstacleRight(targetlabel_, right))
        {
            if (left[1] != right[1])
            {
                weight = fabs(right[1] - left[1]) * resolution_;
            }
            else
            {
                weight = resolution_;
            }
        }
    }

    void ConnectAeras::ComputeLength(int targetlabel_, double &length)
    {
        int up[2], down[2];
        if (IndexInObstacleUp(targetlabel_, up) && IndexInObstacleDown(targetlabel_, down))
        {
            if (down[0] != up[0])
            {
                length = fabs(down[0] - up[0]) * resolution_;
            }
            else
            {
                length = resolution_;
            }
        }
    }

    void ConnectAeras::ComputeDensity()
    {
        vector<int> label_;
        int num = 0;
        label_ = GetAllLabelInMap();
        for (int i = 0; i < label_.size(); i++)
        {
            for (int t = 0; t < Label.size(); t++)
            {
                for (int p = 0; p < Label[t].size(); p++)
                {
                    if (Label[t][p] == label_[i])
                    {
                        num += 1;
                    }
                }
            }
        }

        float subDensity = num * resolution_ * resolution_;
        float mapDensity = (Map.size() * resolution_) * (Map[0].size() * resolution_);
        densityOfObstacle = subDensity / mapDensity;
    }

    void ConnectAeras::ComputeHeight(int targetlabel_, float &height)
    {
        float tmp = 0;
         for (int i = 0; i < Label.size(); i++)
        {
            for (int j = 0; j < Label[i].size(); j++)
            {
                    if (Label[i][j] == targetlabel_)
                    {
                        if (tmp <= fabs(Map[i][j]))
                        {
                            tmp = Map[i][j];
                        }
                        else
                        {
                            continue;
                        }
                    }
            }
        }
          height = tmp;
    }

    void ConnectAeras::ComputeDistanceX(int targetlabel_, double &distanceX)
    {
        double x, y;
        GetCenterPositionInSubmapX(targetlabel_, x);
        GetCenterPositionInSubmapY(targetlabel_, y);
        float centerXInMap, centerYInMap;
        distanceX = y * sin(angle) + x * cos(angle);
    }
    

    void ConnectAeras::DetectObstacleType(int num)
    {
        if (fabs(ObstacleSubMap[num].height) < obstacleHeightLevel1)
        {
            if (ObstacleSubMap[num].height > 0)
            {
                if (ObstacleSubMap[num].wide < obstacleWideLevel1 && ObstacleSubMap[num].length < obstacleLengthLevel1)
                {
                    ObstacleSubMap[num].typeOfObstacle = "smallrock";
                }
                else
                {
                    ObstacleSubMap[num].typeOfObstacle = "smallrock";
                }
            }
            else
            {
                if (ObstacleSubMap[num].wide < obstacleWideLevel1 && ObstacleSubMap[num].length < obstacleLengthLevel1)
                {
                    ObstacleSubMap[num].typeOfObstacle = "largecrater";
                }
                else
                {
                    ObstacleSubMap[num].typeOfObstacle = "largecrater";
                }
            }
        }
        else if (fabs(ObstacleSubMap[num].height) >= obstacleHeightLevel1 && fabs(ObstacleSubMap[num].height) <= obstacleHeightLevel2)
        {
            if (ObstacleSubMap[num].height > 0)
            {
                ObstacleSubMap[num].typeOfObstacle = "rock";
                
            }
            else
            {
                ObstacleSubMap[num].typeOfObstacle = "largecrater";
                
            }
        }
        else if (fabs(ObstacleSubMap[num].height) > obstacleHeightLevel2 && fabs(ObstacleSubMap[num].height) < obstacleHeightLevel3)
        {
            if (ObstacleSubMap[num].height > 0)
            {
                ObstacleSubMap[num].typeOfObstacle = "largerock";
            }
            
            else
            {
                ObstacleSubMap[num].typeOfObstacle = "largecrater";
                
            }
        }
        else if (fabs(ObstacleSubMap[num].height) >= obstacleHeightLevel3)
        {
            if (ObstacleSubMap[num].height > 0)
            {
                if (ObstacleSubMap[num].angleIN >= obstacleAngleLevel1)
                {
                    ObstacleSubMap[num].typeOfObstacle = "hill";
                }
                else if (ObstacleSubMap[num].angleIN > 0 && ObstacleSubMap[num].angleIN < obstacleAngleLevel1 && ObstacleSubMap[num].wide >= obstacleWideLevel3 && ObstacleSubMap[num].length >= obstacleLengthLevel3)
                {
                    ObstacleSubMap[num].typeOfObstacle = "slope";
                }
                else
                {
                    ObstacleSubMap[num].typeOfObstacle = "largerock";
                }
            }
            else
            {
                ObstacleSubMap[num].typeOfObstacle = "largecrater";
                
            }
        }
        else{
            ObstacleSubMap[num].typeOfObstacle = "WeiZhiObstacle";
            ROS_INFO("Height is : %.4f",fabs(ObstacleSubMap[num].height));
            ROS_INFO("Height is : %.4f",ObstacleSubMap[num].height);
        }
    }

    void ConnectAeras::SetObstacleTypeData(float heightlevel1, float heightlevel2, float heightlevel3, double wideLevel1, double wideLevel2, double wideLevel3, double lengthLevel1, double lengthLevel2, double lengthLevel3, double anglelevel1)
    {
        obstacleHeightLevel1 = heightlevel1; //wheel r
        obstacleHeightLevel2 = heightlevel2; //wheel 2*r
        obstacleHeightLevel3 = heightlevel3; //rover height
        obstacleWideLevel1 = wideLevel1;
        obstacleWideLevel2 = wideLevel2;
        obstacleWideLevel3 = wideLevel3;
        obstacleLengthLevel1 = lengthLevel1;
        obstacleLengthLevel2 = lengthLevel2;
        obstacleLengthLevel3 = lengthLevel3;
        obstacleAngleLevel1 = anglelevel1; // 45
    }

    void ConnectAeras::SetRobotFrameID(string robotframe)
    {
        robotFrameID = robotframe;
    }

    void ConnectAeras::SetGridMapFrameID(string gridmapframe)
    {
        gridmapFrameID = gridmapframe;
    }

    //Get the rosbot motion path position in the world frame.
    void ConnectAeras::ReadRobotPath(vector<vector<double>> position)
    {
        for (int i = 0; i < position.size(); i++)
        {
            vector<double> tmp;
            for (int j = 0; j < position[i].size(); j++)
            {
                tmp.push_back(position[i][j]);
            }
            pathPosition.push_back(tmp);
        }
    }

    //Get the robot position in the world frame.
    void ConnectAeras::SetRobotPosition(double x, double y)
    {
        robotInWorldX = x;
        robotInWorldY = y;
    }

    void ConnectAeras::SetWheelPath()
    {
        double right, left;
        for (int i = 0; i < pathPosition.size(); i++)
        {
            vector<double> tmpRight, tmpLeft;
            right = pathPosition[i][1] - (robotWide * 0.5);
            left = pathPosition[i][1] + (robotWide * 0.5);
            tmpRight.push_back(pathPosition[i][0]);
            tmpRight.push_back(right);
            tmpLeft.push_back(pathPosition[i][0]);
            tmpLeft.push_back(left);
            rightWheelPath.push_back(tmpRight);
            leftWheelPath.push_back(tmpLeft);
        }
    }

    vector<vector<int>> ConnectAeras::GetObstacleCellIndex(int label_)
    {
        vector<vector<int>> indexXY;
        for (int i = 0; i < Label.size(); i++)
        {
            vector<int> tmp;
            for (int j = 0; j < Label[i].size(); j++)
            {
                if (Label[i][j] == label_)
                {
                    tmp.push_back(i);
                    tmp.push_back(j);
                    indexXY.push_back(tmp);
                    tmp.clear();
                }
            }
        }
        return indexXY;
    }

    vector<int> ConnectAeras::GetSubMapRowsAndColunms(int label_)
    {
        vector<vector<int>> indexXY;
        vector<int> Row, Col, tmp;
        int row_1, row_2, col_1, col_2;
        indexXY = GetObstacleCellIndex(label_);
        for (int i = 0; i < indexXY.size(); i++)
        {
            Row.push_back(indexXY[i][0]);
            Col.push_back(indexXY[i][1]);
        }
        row_1 = IndexMax(Row);
        row_2 = IndexMin(Row);
        col_1 = IndexMax(Col);
        col_2 = IndexMin(Col);
        
        if ((row_1 != row_2) && (col_1 != col_2))
        {
            tmp.push_back(row_1 - row_2 + 1);
            tmp.push_back(col_1 - col_2 + 1);
            return tmp;
        }

        else if ((row_1 == row_2) && (col_1 != col_2))
        {
            tmp.push_back(1);
            tmp.push_back(col_1 - col_2 + 1);
            return tmp;
        }
        else if ((row_1 != row_2) && (col_1 == col_2))
        {
            tmp.push_back(row_1 - row_2 + 1);
            tmp.push_back(1);
            return tmp;
             
        }
        else if ((row_1 == row_2) && (col_1 == col_2))
        {
            tmp.push_back(1);
            tmp.push_back(1);
            return tmp;
        }
    }

    void ConnectAeras::UpdateNewMap()
    {
        vector<int> label = GetAllLabelInMap();
        NewMap zero;
        zero.height = 0;
        zero.label = 0;
        zero.index[0] = 0;
        zero.index[1] = 0;
        zero.position[0] = 0;
        zero.position[1] = 0;
        for (int i = 0; i < Map.size(); i++)
        {
            vector<NewMap> tmp;
            for (int j = 0; j < Map[i].size(); j++)
            {
                tmp.push_back(zero);
            }
            newMap.push_back(tmp);
        }

        for (int i = 0; i < label.size(); i++)
        {

            CellDataInWorld(label[i]);
        }
    }

    void ConnectAeras::CellDataInWorld(int label_)
    {
        vector<vector<int>> index_ = GetObstacleCellIndex(label_);
        for (int i = 0; i < index_.size(); i++)
        {
            newMap[index_[i][0]][index_[i][1]].position[0] = (Map.size() * resolution_) - (index_[i][0] * resolution_) + robotInWorldX;
            newMap[index_[i][0]][index_[i][1]].position[1] = (Map[0].size() * resolution_) - (index_[i][1] * resolution_) + robotInWorldY;
            newMap[index_[i][0]][index_[i][1]].height = Map[index_[i][0]][index_[i][1]];
            newMap[index_[i][0]][index_[i][1]].label = label_;
            newMap[index_[i][0]][index_[i][1]].index[0] = index_[i][0];
            newMap[index_[i][0]][index_[i][1]].index[1] = index_[i][1];
        }
    }

    void ConnectAeras::Obstacle()
    {
        UpdateNewMap();
        if (GetTargetlabel())
        {
            //ROS_INFO("GetTargetLabel.size is : %d",targetLabel.size());
            ObstacleSubMap.resize(targetLabel.size());
            for (int i = 0; i < targetLabel.size(); i++)
            {
                float height;
                double x, y;
                ComputeHeight(targetLabel[i], height);
                ObstacleSubMap[i].height = height;
                ObstacleSubMap[i].label = targetLabel[i];
                ObstacleCenterPositionInWorld(targetLabel[i], i);
                ObstacleSubMapInworld(targetLabel[i], i);
                ObstacleWide(targetLabel[i], i);
                ObstacleLength(targetLabel[i], i);
                ObstaclePlaneFitting(targetLabel[i], i);
                PositionRelation(i);
                DetectObstacleType(i);
                OutputModeID(i);
                ObstacleDistanceWithRobot(targetLabel[i], i);
            }
        }
        else
        {
            ROS_ERROR("Not Label In Map");
        }
    }

    void ConnectAeras::ObstacleLength(int label_, int num)
    {
        ComputeLength(label_, ObstacleSubMap[num].length);
    }

    void ConnectAeras::ObstacleWide(int label_, int num)
    {
        ComputeWeight(label_, ObstacleSubMap[num].wide);
    }

    void ConnectAeras::ObstacleDistanceWithPath(int label_, int num, double &x, double &y)
    {
        double xO = ObstacleSubMap[num].centerPosition[0]; // x
        double yO = ObstacleSubMap[num].centerPosition[1]; // y
        vector<double> d;
        double tmpx, tmpy;
        for (int i = 0; i < pathPosition.size(); i++)
        {
            d.push_back((pow((xO - pathPosition[i][0]), 2) + pow((yO - pathPosition[i][1]), 2)));
        }
        double tmp = d[0];
        for (int i = 0; i < d.size(); i++)
        {
            if (tmp < d[i])
            {
                tmp = d[i];
                tmpx = pathPosition[i][0];
                tmpy = pathPosition[i][1];
            }
        }
        ObstacleSubMap[num].distanceWithPath = tmp;
        x = tmpx;
        y = tmpy;
    }

   void ConnectAeras::ObstacleCenterPositionInWorld(int label_, int num)
    {
        double x, y;
        GetCenterPositionInSubmapX(label_, x);
        GetCenterPositionInSubmapY(label_, y);
        ObstacleSubMap[num].centerPosition[0] = robotInWorldX + x;
        ObstacleSubMap[num].centerPosition[1] = robotInWorldY + y;
    }

    void ConnectAeras::ObstacleSubMapInworld(int label_, int num)
    {
        vector<int> size;
        size = GetSubMapRowsAndColunms(label_);
        int i_begin[2], j_begin[2];
        IndexInObstacleUp(label_, i_begin);
        IndexInObstacleLeft(label_, j_begin);
        int index_begin[2];
        index_begin[0] = i_begin[0];
        index_begin[1] = j_begin[1];
        NewMap zeros;
        zeros.height = 0;
        zeros.index[0] = 0;
        zeros.index[1] = 0;
        zeros.label = 0;
        zeros.position[0] = 0;
        zeros.position[1] = 0;
        for (int i = 0; i < size[0]; i++)
        {
            vector<NewMap> tmp;
            for (int j = 0; j < size[1]; j++)
            {
                tmp.push_back(zeros);
            }
            ObstacleSubMap[num].subMap.push_back(tmp);
        }
        cout << ObstacleSubMap[num].subMap[0].size() << endl;
        for (int i = 0; i < newMap.size(); i++)
        {

            for (int j = 0; j < newMap[i].size(); j++)
            {
                if (newMap[i][j].label == label_)
                {
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].height = newMap[i][j].height;
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].index[0] = newMap[i][j].index[0];
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].index[1] = newMap[i][j].index[1];
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].position[0] = newMap[i][j].position[0];
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].position[1] = newMap[i][j].position[1];
                    ObstacleSubMap[num].subMap[i - index_begin[0]][j - index_begin[1]].label = newMap[i][j].label;
                }
            }
        }
    }

   void ConnectAeras::ObstaclePositionRelation(int num)
    {
        vector<vector<double>> position;
        vector<double> tmp;
        for (int i = 0; i < ObstacleSubMap[num].subMap.size(); i++)
        {
            for (int j = 0; j < ObstacleSubMap[num].subMap[i].size(); j++)
            {
                if(ObstacleSubMap[num].subMap[i][j].label == ObstacleSubMap[num].label)
                {
                    tmp.push_back(ObstacleSubMap[num].subMap[i][j].position[0]);
                    tmp.push_back(ObstacleSubMap[num].subMap[i][j].position[1]);
                }
                else{
                    continue;
                }
            }
            
        }

        // Get the set of y-values for the upper and lower bounds of the obstacle submap, 
        // and the x-values for the upper and lower bounds. 
        // Use the x-values to extract the coordinates of the wheel trajectory.
        for (int i = 0; i < tmp.size(); i+=2)
        {
            vector<double> t;
            t.push_back(tmp[i]);
            t.push_back(tmp[i + 1]);
            position.push_back(t);
            t.clear();
        }

        double y_left = ObstacleSubMap[num].centerPosition[1] + (ObstacleSubMap[num].subMap[0].size() * 0.5 * resolution_);
        double y_right = ObstacleSubMap[num].centerPosition[1] - (ObstacleSubMap[num].subMap[0].size() * 0.5 * resolution_);
        double x_up = ObstacleSubMap[num].centerPosition[0] + (ObstacleSubMap[num].subMap.size() * 0.5 * resolution_);
        double x_down = ObstacleSubMap[num].centerPosition[0] - (ObstacleSubMap[num].subMap.size() * 0.5 * resolution_);
        vector<vector<double>> right, left;
        vector<double> tmp1, tmp2;
        for (int i = 0; i < rightWheelPath.size(); i++)
        {
            if(rightWheelPath[i][0] < x_up && rightWheelPath[i][0] > x_down)
            {
                tmp1.push_back(rightWheelPath[i][0]);
                tmp1.push_back(rightWheelPath[i][1]);
                tmp2.push_back(leftWheelPath[i][0]);
                tmp2.push_back(leftWheelPath[i][1]);
            }
            else
            {

            }
        }

        
        if(!tmp1.empty())
        {
            for (int i = 0; i < tmp1.size(); i += 2)
            {
                vector<double> tt1, tt2;
                tt1.push_back(tmp1[i]);
                tt1.push_back(tmp1[i]);
                tt2.push_back(tmp2[i]);
                tt2.push_back(tmp2[i]);
                right.push_back(tt1);
                left.push_back(tt2);
            }

            double rightMin = right[0][1];
            double rightMax = right[0][1];
            double leftMin = left[0][1];
            double leftMax = left[0][1];

            for (int i = 0; i < right.size(); i++)
            {
                if(rightMin > right[i][1])
                {
                    rightMin = right[i][1];
                }
                if(rightMax < right[i][1])
                {
                    rightMax = right[i][1];
                }
                if(leftMin > left[i][1])
                {
                    leftMin = left[i][1];
                }
                if(leftMax < left[i][1])
                {
                    leftMax = left[i][1];
                }
            }

            ROS_INFO("ObstacleSubMap[num].centerPosition[0] is : %.4f, ObstacleSubMap[num].centerPosition[1] is : %.4f", ObstacleSubMap[num].centerPosition[0], ObstacleSubMap[num].centerPosition[1]);
            ROS_INFO("leftMax is :%.4f, leftMin is :%.4f, y_left is :%.4f, y_right is :%.4f", leftMax, leftMin, y_left, y_right);
            ROS_INFO("rightMax is :%.4f, rightMin is :%.4f, y_left is :%.4f, y_right is :%.4f", rightMax, rightMin, y_left, y_right);
            if (rightMin >= y_right && (rightMax - 0.2) <= y_left && (leftMin - 0.2) > y_left)
            {
                ObstacleSubMap[num].positionRelation = "RightWheel";
            }
            else if(leftMin >= y_right && (leftMax - 0.2) <= y_left && (rightMax + 0.2) < y_right)
            {
                ObstacleSubMap[num].positionRelation = "LeftWheel";
            }
            else if(leftMax <= y_left && rightMin >= y_right)
            {
                ObstacleSubMap[num].positionRelation = "AcrossWheel";
            }
            else if((leftMin - 0.2) > y_left && (rightMax + 0.2) < y_right)//两个不等号改变了
            {
                ObstacleSubMap[num].positionRelation = "InsideWheel";
            } 
            else
            {
                ObstacleSubMap[num].positionRelation = "OutOfScope";
            }
        }
        else
        {
            ObstacleSubMap[num].positionRelation = "OutOfScope";
        }     
    }

    vector<ObstacleMap> ConnectAeras::GetObstacleData()
    {
        return ObstacleSubMap;
    }

    int ConnectAeras::IndexMax(vector<int> index_)
    {
        int tmp = index_[0];
        for (int i = 0; i < index_.size(); i++)
        {
            if (tmp < index_[i])
            {
                tmp = index_[i];
            }
        }
        return tmp;
    }

    int ConnectAeras::IndexMin(vector<int> index_)
    {
        int tmp = index_[0];
        for (int i = 0; i < index_.size(); i++)
        {
            if (tmp > index_[i])
            {
                tmp = index_[i];
            }
        }
        return tmp;
    }

    void ConnectAeras::ObstaclePlaneFitting(int label_, int num)
    {
        vector<double> x, y, z;
        double tmp;
        double a, b, c;
        double anglePlaneOut, anglePlaneIn;
        for (int i = 0; i < ObstacleSubMap[num].subMap.size(); i++)
        {
            for (int j = 0; j < ObstacleSubMap[num].subMap[i].size(); j++)
            {
                if (ObstacleSubMap[num].subMap[i][j].label == label_)
                {
                    x.push_back(ObstacleSubMap[num].subMap[i][j].position[0]);
                    y.push_back(ObstacleSubMap[num].subMap[i][j].position[1]);
                    tmp = ObstacleSubMap[num].subMap[i][j].height;
                    z.push_back(tmp);
                }
            }
        }

        int n = x.size();
        if (gFittingPlane(x, y, z, n, a, b, c))
        {
            double result = 1.0 / (sqrt(pow(a, 2) + pow(b, 2) + 1));
            ROS_INFO("Plane Fitting OK");
            anglePlaneIn = acos(result);
        }

        ObstacleSubMap[num].angleIN = anglePlaneIn;
        ObstacleSubMap[num].angleOUT = anglePlaneOut;
    }

    bool ConnectAeras::gFittingPlane(vector<double> x, vector<double> y, vector<double> z, int n, double &a, double &b, double &c)
    {
        int i;
        double x1, x2, y1, y2, z1, xz, yz, xy, r;
        x1 = x2 = y1 = y2 = z1 = xz = yz = xy = 0;
        for (i = 0; i < n; i++)
        {
            x1 += x[i];
            x2 += x[i] * x[i];
            xz += x[i] * z[i];

            y1 += y[i];
            y2 += y[i] * y[i];
            yz += y[i] * z[i];

            z1 += z[i];
            xy += x[i] * y[i];
        }

        Eigen::Matrix3d m1 = Eigen::Matrix3d::Random(3, 3);
        Eigen::Matrix3d m2 = Eigen::Matrix3d::Random(3, 3);
        Eigen::Matrix3d m3 = Eigen::Matrix3d::Random(3, 3);
        Eigen::Matrix3d m = Eigen::Matrix3d::Random(3, 3);

        m(0, 0) = x2;
        m(0, 1) = xy;
        m(0, 2) = x1;
        m(1, 0) = xy;
        m(1, 1) = y2;
        m(1, 2) = y1;
        m(2, 0) = x1;
        m(2, 1) = y1;
        m(2, 2) = n;
        r = m.determinant();
        if (!r)
            return false;

        m1(0, 0) = xz;
        m1(0, 1) = xy;
        m1(0, 2) = x1;
        m1(1, 0) = yz;
        m1(1, 1) = y2;
        m1(1, 2) = y1;
        m1(2, 0) = z1;
        m1(2, 1) = y1;
        m1(2, 2) = n;
        a = m1.determinant() / r;

        m2(0, 0) = x2;
        m2(0, 1) = xz;
        m2(0, 2) = x1;
        m2(1, 0) = xy;
        m2(1, 1) = yz;
        m2(1, 2) = y1;
        m2(2, 0) = x1;
        m2(2, 1) = z1;
        m2(2, 2) = n;
        b = m2.determinant() / r;

        m3(0, 0) = x2;
        m3(0, 1) = xy;
        m3(0, 2) = xz;
        m3(1, 0) = xy;
        m3(1, 1) = y2;
        m3(1, 2) = yz;
        m3(2, 0) = x1;
        m3(2, 1) = y1;
        m3(2, 2) = z1;
        c = m3.determinant() / r;

        return true;
    }

    void ConnectAeras::PositionRelation(int num)
    {
        ROS_INFO("roverAndObstaclePositionInY is : %.4f",roverAndObstaclePositionInY);
        double p1 = fabs(fabs(roverAndObstaclePositionInY) - (ObstacleSubMap[num].wide * 0.5));
        double p2 = fabs(fabs(roverAndObstaclePositionInY) + (ObstacleSubMap[num].wide * 0.5));
        double tmp = robotWide * 0.5;
        if (tmp >= p1 && tmp <= p2 && roverAndObstaclePositionInY < 0)
        {
            ObstacleSubMap[num].positionRelation = "RightWheel";
        }
        else if (tmp >= p1 && tmp <= p2 && roverAndObstaclePositionInY > 0)
        {
            ObstacleSubMap[num].positionRelation = "LeftWheel";
        }
        else if (tmp >= p2 && ObstacleSubMap[num].wide < robotWide)
        {
            ObstacleSubMap[num].positionRelation = "InsideWheel";
        }
        else if (tmp <= p1 && ObstacleSubMap[num].wide >= robotWide)
        {
            ObstacleSubMap[num].positionRelation = "AcrossWheel";
        }
        else
        {
            ObstacleSubMap[num].positionRelation = "OutOfScope";
        }
    }

    void ConnectAeras::OutputModeID(int num)
    {
        if(ObstacleSubMap[num].typeOfObstacle == "largecrater" || ObstacleSubMap[num].typeOfObstacle == "hill" || ObstacleSubMap[num].typeOfObstacle == "largerock")
        {
            if(ObstacleSubMap[num].positionRelation == "OutOfScope")
            {
                ObstacleSubMap[num].modeID = "WRM";
            }
            else if(ObstacleSubMap[num].positionRelation == "InsideWheel")
            {
                ObstacleSubMap[num].modeID = "BLM";
            }
             else if(ObstacleSubMap[num].positionRelation == "AcrossWheel")
            {
                ObstacleSubMap[num].modeID = "DWLM";
            }
            else
            {
                ObstacleSubMap[num].modeID = "WCM";
            }
            
        }
        else if(ObstacleSubMap[num].typeOfObstacle == "smallcrater" || ObstacleSubMap[num].typeOfObstacle == "smallrock")
        {
            ObstacleSubMap[num].modeID = "WRM";
        }
        else if(ObstacleSubMap[num].typeOfObstacle == "slope")
        {
            if(ObstacleSubMap[num].positionRelation == "RightWheel" || ObstacleSubMap[num].positionRelation == "LeftWheel" || ObstacleSubMap[num].positionRelation == "InsideWheel")
            {
                ObstacleSubMap[num].modeID = "WCM";
            }
            else if(ObstacleSubMap[num].positionRelation == "AcrossWheel")
            {
                ObstacleSubMap[num].modeID = "BCM";
            }
            else if(ObstacleSubMap[num].positionRelation == "OutOfScope")
            {
                ObstacleSubMap[num].modeID = "WRM";
            }
        }
        else if(ObstacleSubMap[num].typeOfObstacle == "crater" || ObstacleSubMap[num].typeOfObstacle == "rock")
        {
            if(ObstacleSubMap[num].positionRelation == "RightWheel")
            {
                ObstacleSubMap[num].modeID = "SRWLM";
                
            }
            else if(ObstacleSubMap[num].positionRelation == "LeftWheel")
            {
                ObstacleSubMap[num].modeID = "SLWLM";
               
            }
            else if(ObstacleSubMap[num].positionRelation == "InsideWheel")
            {
                ObstacleSubMap[num].modeID = "BLM";
               
            }
            else if(ObstacleSubMap[num].positionRelation == "AcrossWheel")
            {
                ObstacleSubMap[num].modeID = "DWLM";
               
            }
            else if(ObstacleSubMap[num].positionRelation == "OutOfScope")
            {
                ObstacleSubMap[num].modeID = "WRM";
            }
            else
            {
                ObstacleSubMap[num].modeID = "WRM";
            }
        }

    }

    void ConnectAeras::ObstacleDistanceWithRobot(int label_, int num)
    {
        double x, y;
        ComputeDistanceX(label_, x);
        ComputeDistanceY(label_, y);
        ObstacleSubMap[num].distanceWithRobotX = x;
        ObstacleSubMap[num].distanceWithRobotY = y;
    }

    vector<ObstacleMap> ConnectAeras::FirstLayer()
    {
        vector<ObstacleMap> targetObstacle;
        Obstacle();
        if (!ObstacleSubMap.empty())
        {
            for (int i = 0; i < ObstacleSubMap.size(); i++)
            {
                targetObstacle.push_back(ObstacleSubMap[i]);
            }
            return targetObstacle;
        }
        else
        {
            ROS_ERROR("Obstacle Sub Map is empty");
        }
    }

    void ConnectAeras::MinDistance(double arr[5], int &index_)
    {
        double tmp=arr[0];
        for (int i = 0; i < 5; i++)
        {
           if(tmp > arr[i])
           {
               tmp = arr[i];
                index_ = i;
           }
           else
           {
               continue;
           }
        }
        return;
    }

    void ConnectAeras::MinDistance(vector<double> v,int &index_)
    {
        double tmp=v[0];
        for (int i = 0; i < v.size(); i++)
        {
           if(tmp > v[i])
           {
               tmp = v[i];
                index_ = i;
           }
           else
           {
               continue;
           }
        }
        return;
    }

    int ConnectAeras::IsAppearObstacle()
    {
        double distancedata[5] = {0,0,0,0,0};
        double distancedataX[5] ={0,0,0,0,0};
        double distancedataY[5] = {0,0,0,0,0};
        int obstacleID[5]={1,2,3,4,5};
        int num=0;
        vector<double> tmp;
        vector<double> tmp1;
        vector<double> TMP;
        vector<int> stateVector;
        int stateID = 0;
        
        for (int i = 0; i < 5; i++)
        {
            distancedata[i] = DistanceNew(roverpositionX, obstaclePosition[i].positionx, roverpositionY, obstaclePosition[i].positiony);
            distancedataX[i] = DistanceNewX(roverpositionX, obstaclePosition[i].positionx);
            distancedataY[i] = DistanceNewY(roverpositionY, obstaclePosition[i].positiony);
        }
        for (int j = 0; j < 5; j++)
        {
            ROS_INFO("distancedataX[%d] is: %.4f",j,distancedataX[j]);
            if (distancedataX[j] <= (marginDX + 0.722) && (distancedataX[j] - 0.722)> 0)
            {
                tmp.push_back(distancedataY[j]);
                tmp1.push_back(distancedataX[j]);
                stateVector.push_back(obstacleID[j]);
                TMP.push_back(distancedata[j]);

                //roverAndObstaclePositionInY = distancedataY[j];
                //return true;
            }
            else
            {
                    continue;
            }
        }
        ROS_INFO("tmp size is : %d",tmp.size());
        if (!tmp.empty())
        {
            MinDistance(tmp,num);
            roverAndObstaclePositionInY = tmp[num]*cos(roverAngleYaw)-tmp1[num]*sin(roverAngleYaw);
            stateID = stateVector[num];
            roverAndObstacleDistance = TMP[num];
            ROS_INFO("roverAndObstaclePositionInY is : %.4f",roverAndObstaclePositionInY);
            if (fabs(roverAndObstaclePositionInY) <= 1.05)
            {
                ROS_ERROR("IF IN?");
                if (stateID == 3 || stateID == 4)
                {
                    if (fabs(roverAngleYaw) >= 3.05)
                    {
                        return stateID;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                     return stateID;
                }
            }
            else
            {
                return 0;
            }
            
        }
        else
        {
            return 0;
        }
        
    }

    void ConnectAeras::SetRoverAngle(double roverrow, double roverpitch, double roveryaw)
    {
        roverAngleRow = roverrow;
        roverAnglePitch = roverpitch;
        roverAngleYaw = roveryaw;
        return;
    }

    void ConnectAeras::SetRoverPosition(double x, double y, double z)
    {
        // Robot absolute position (vision capture system)
        roverpositionX = x;
        roverpositionY = y;
        roverpositionZ = z;
        return;
    }

    double ConnectAeras::DistanceNewX(double pointx, double child_pointx)
    {
        ROS_INFO("Yaw Angle is : %.4f",roverAngleYaw);
        double result = child_pointx - pointx;
        if (fabs(roverAngleYaw) <= 1.57)
        {
            return result;
        }
        else if (fabs(roverAngleYaw) >= 1.57)
        {
            return -result;
        }
    }

    double ConnectAeras::DistanceNewY(double pointy, double child_pointy)
    {
        double result = child_pointy - pointy;
        return result;
    }

    double ConnectAeras::DistanceNew(double pointx, double child_pointx, double pointy, double child_pointy)
    {
        double result = sqrt(pow((pointx - child_pointx), 2) + pow((pointy - child_pointy), 2));
        return result;
    }

    ///////////////////////////////////test//////////////////////////////////
    string ConnectAeras::GetModeID()
    {
        return ObstacleSubMap[0].modeID;
    }
    string ConnectAeras::GetType()
    {
        return ObstacleSubMap[0].typeOfObstacle;
    }

     double ConnectAeras::GetRoverAndObstacleDistance()
     {
         return roverAndObstacleDistance;
     }

}
