/*
 * SecondLayer.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "elevation_mapping/SecondLayer.h"

using namespace std;
using namespace Eigen;

namespace SecondLayer
{
    SecondLayer::SecondLayer()
    {
        anchorPoint = 4;
        numberOfSharpBumps = 0;
        planeWight_1 = 1;
        planeWight_2 =0.8;
        planeWight_3 = 0.5;
        planeWight_4 = 0;
    }

    void SecondLayer::SetAnchorPoint(int anchorPointValue)
    {
        anchorPoint = anchorPointValue;
    }

    void SecondLayer::SetTargetObstacleMap(ObstacleMap obstacle)
    {
        Obstacle = obstacle;
    }

    void SecondLayer::SetMarginOfSharpness(float sharpness)
    {
        sharpnessMargin = sharpness;
    }

    void SecondLayer::SetSharpnessradius(double radius)
    {
        sharpnessRadiusMargin = radius;
    }

    void SecondLayer::SetRobotWide(double robotwide_)
    {
        robotWide = robotwide_;
    }

    void SecondLayer::SetResolution(double resolution_)
    {
        resolution = resolution_;
    }

    void SecondLayer::SetAreaInPlane(double wideOfPlane)
    {
        int result = 0;
        double real;
        result = wideOfPlane / resolution;
        real = wideOfPlane / resolution;
        if((real-result) >= 0.5)
        {
            result += 1;
        }
        areaInPlane = result;
    }

    void SecondLayer::SetTargetVector(vector<double> targetVector)
    {
        targetSequence = targetVector;
    }

    void SecondLayer::SetGRAParams(double margin_, vector<double> weight_, double row_)
    {
        graMargin = margin_;
        graweight = weight_;
        graRow = row_;
    }

    void SecondLayer::SetSNMargin(double margin_1, double margin_2, double margin_3)
    {
        SNmargin_1 = margin_1;
        SNmargin_2 = margin_2;
        SNmargin_3 = margin_3;
    }

    void SecondLayer::SearchMin(float &max, float &min)
    {
        max = Obstacle.subMap[0][0].height;
        min = Obstacle.subMap[0][0].height;
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            for (int j = 0; j < Obstacle.subMap[i].size(); j++)
            {
                if(max < Obstacle.subMap[i][j].height)
                {
                    max = Obstacle.subMap[i][j].height;
                }
                else if(min > Obstacle.subMap[i][j].height)
                {
                    min = Obstacle.subMap[i][j].height;
                }
                else continue;
            }
            
        }
        
    }

    float SecondLayer::HeightMappingCell(float height)
    {
        float cell, heightMax, heightMin;
        SearchMin(heightMax, heightMin);
        cell = (255 / (heightMax - heightMin)) * (height - heightMin);
        return cell;
    }

    void SecondLayer::TurnMapType()
    {
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            vector<float> tmp;
            for (int j = 0; j < Obstacle.subMap[i].size(); j++)
            {
                float cell = HeightMappingCell(Obstacle.subMap[i][j].height);
                tmp.push_back(cell);
            }
            Map.push_back(tmp);
        }
    }

    bool SecondLayer::Edge_(vector< vector<float> > test_Map)
    {
        for (int i = 0; i < test_Map.size() + 2; i++)
        {
            vector<float> raw;
            if (i == 0)
            {
                raw.assign(test_Map[i].begin(), test_Map[i].end());
                raw.insert(raw.begin(), test_Map[i][0]);
                raw.push_back(test_Map[i][test_Map[i].size() - 1]);
            }
            else if (i == test_Map.size() + 1)
            {
                raw.assign(test_Map[i-2].begin(), test_Map[i-2].end());
                raw.insert(raw.begin(), test_Map[i-2][0]);
                raw.push_back(test_Map[i-2][test_Map[i-2].size() - 1]);
            }
            else
            {
                raw.assign(test_Map[i-1].begin(), test_Map[i-1].end());
                raw.insert(raw.begin(), test_Map[i - 1][0]);
                raw.push_back(test_Map[i - 1][test_Map[i - 1].size() - 1]);
            }
            Map_And_Edge.push_back(raw);
            
        }
        return true;
    }

    void SecondLayer::ConvalutionSynthesis()
    {
        if (Edge_(Map))
        {
            for (int i = 1; i < Map_And_Edge.size() - 1; i++)
                {
                    vector<float> value;
                    for (int j = 1; j < Map_And_Edge[i].size() - 1; j++)
                    {
                        Index['U'] = Map_And_Edge[i - 1][j]; // Upper value
                        Index['D'] = Map_And_Edge[i + 1][j]; // Lower value
                        Index['L'] = Map_And_Edge[i][j - 1]; // Left value
                        Index['R'] = Map_And_Edge[i][j + 1]; // Right value
                        Index['C'] = Map_And_Edge[i][j]; // Center anchor point
                        value.push_back(Convalution_Compute(Index));
                    }
                    convolution_map_.push_back(value);
                }
        }
        

    }

    float SecondLayer::Convalution_Compute(map<char, float> cell)
    {
        float value = anchorPoint * cell['C'] - cell['U'] - cell['D'] - cell['L'] - cell['R'];
        return value;
    }

    void SecondLayer::Sharpen()
    {
        for (int i = 0; i < Map.size(); i++)
        {

            for (int j = 0; j < Map[i].size(); j++)
            {
                convolution_map_[i][j] += Map[i][j];
                if(convolution_map_[i][j] >= 255)
                {
                    convolution_map_[i][j] = 255;
                }
            }
        }

    }

    vector<int> SecondLayer::SearchMaxIndex(int label_, float maxValue)
    {
        vector<int> index;
        for (int i = 0; i < convolution_map_.size(); i++)
        {
            for (int j = 0; j < convolution_map_[i].size(); j++)
            {
                if(convolutionLabelSubMap[i][j] == label_ && convolution_map_[i][j] == maxValue)
                {
                    index.push_back(i);
                    index.push_back(j);
                    return index;
                }
            }
            
        }
        
    }

    void SecondLayer::Convolution()
    {
        TurnMapType();
        ConvalutionSynthesis();
        CCL.Set_Margin(500);
        CCL.Set_Map(convolution_map_);
        
        CCL.TWO_PASS();
    
        convolutionLabelSubMap = CCL.Get_ConnectMap();

        // Get the maximum convolution value among the bumps in the convolution map
        vector<int> allLabel = CCL.GetAllLabelInMap();
        
        vector<int> sharpnessMaxValueIndex;
        float convolutionMax;
        vector<float> sharpConvolution;
        for (int i = 0; i < allLabel.size(); i++)
        {
            CCL.ComputeHeight(allLabel[i], convolutionMax);
            sharpConvolution.push_back(convolutionMax);
        }
    }

    void SecondLayer::ConvolutionSubMap(int label_, int num)
    {
        vector<int> size;
        size = CCL.GetSubMapRowsAndColunms(label_);
        int i_begin[2], j_begin[2];
        CCL.IndexInObstacleUp(label_, i_begin);
        CCL.IndexInObstacleLeft(label_, j_begin);
        int index_begin[2];
        index_begin[0] = i_begin[0];
        index_begin[1] = j_begin[1];
        ConvolutionMap zeros;
        zeros.height = 0;
        zeros.index[0] = 0;
        zeros.index[1] = 0;
        zeros.label = 0;
        zeros.convolutionValue = 0;

        for (int i = 0; i < size[0]; i++)
        {
            vector<ConvolutionMap> tmp;
            for (int j = 0; j < size[1]; j++)
            {
                tmp.push_back(zeros);
            }
            convolutionSubMap[num].subMap.push_back(tmp);
        }

        for (int i = 0; i < NewConvolutionMap.size(); i++)
        {

            for (int j = 0; j < NewConvolutionMap[i].size(); j++)
            {
                if(NewConvolutionMap[i][j].label == label_)
                {
                    convolutionSubMap[num].subMap[i-index_begin[0]][j-index_begin[1]].convolutionValue = NewConvolutionMap[i][j].height;
                    convolutionSubMap[num].subMap[i-index_begin[0]][j-index_begin[1]].height = Obstacle.subMap[i][j].height;
                    convolutionSubMap[num].subMap[i-index_begin[0]][j-index_begin[1]].index[0] = NewConvolutionMap[i][j].index[0];
                    convolutionSubMap[num].subMap[i-index_begin[0]][j-index_begin[1]].index[1] = NewConvolutionMap[i][j].index[1];
                    convolutionSubMap[num].subMap[i-index_begin[0]][j-index_begin[1]].label = NewConvolutionMap[i][j].label;
                }
                else
                {
                    continue;
                }
            }
            
        }
    }

    // Update the data within each grid of the obstacle map
    void SecondLayer::UpdateNewConvolutionMap()
    {
        vector<int> label = CCL.GetAllLabelInMap();
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
            NewConvolutionMap.push_back(tmp);
        }
        
        for (int i = 0; i < label.size(); i++)
        {
            
            vector<vector<int>> index_ = CCL.GetObstacleCellIndex(label[i]);
            for (int j = 0; j < index_.size(); j++)
            {
                NewConvolutionMap[index_[j][0]][index_[j][1]].height =  Obstacle.subMap[index_[j][0]][index_[j][1]].height;
                NewConvolutionMap[index_[j][0]][index_[j][1]].label = label[i];
                NewConvolutionMap[index_[j][0]][index_[j][1]].index[0] = index_[j][0];
                NewConvolutionMap[index_[j][0]][index_[j][1]].index[1] = index_[j][1];
            }

        }
    }

    float SecondLayer::ComputeBumpsHeight(int label_)
    {
        float result = 0;
        for (int i = 0; i < NewConvolutionMap.size(); i++)
        {
            for (int j = 0; j < NewConvolutionMap[i].size(); j++)
            {
                if(NewConvolutionMap[i][j].label = label_)
                {
                    if(result <= NewConvolutionMap[i][j].height)
                    {
                        result = NewConvolutionMap[i][j].height;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            
        }
        return result;
    }

    vector<int> SecondLayer::SearchMaxHeightIndex(int label_, float maxHeight)
    {
        vector<int> indexResult;

        for (int i = 0; i < NewConvolutionMap.size(); i++)
        {
            for (int j = 0; j < NewConvolutionMap[i].size(); j++)
            {
                if(NewConvolutionMap[i][j].label == label_ && NewConvolutionMap[i][j].height == maxHeight)
                {
                    indexResult.push_back(i);
                    indexResult.push_back(j);
                    return indexResult;
                }
            }
            
        }
        
    }

    double SecondLayer::SharpBumpsRadius(int label_, int num)
    {
        float bumpsMaxHeight = ComputeBumpsHeight(label_);
        vector<int> sharpnessMaxValueIndex = SearchMaxHeightIndex(label_, bumpsMaxHeight);
        convolutionSubMap[num].height = bumpsMaxHeight;
        convolutionSubMap[num].idex[0] = sharpnessMaxValueIndex[0];
        convolutionSubMap[num].idex[1] = sharpnessMaxValueIndex[1];

        double index[2];
        double radius;
        for (int i = 0; i < convolutionSubMap[num].subMap.size(); i++)
        {
            for (int j = 0; j < convolutionSubMap[num].subMap[i].size(); j++)
            {
                if((bumpsMaxHeight - convolutionSubMap[num].subMap[i][j].convolutionValue) >= sharpnessMargin && (bumpsMaxHeight - convolutionSubMap[num].subMap[i][j].convolutionValue) >0 && convolutionSubMap[num].subMap[i][j].label ==label_)
                {
                    index[0] = convolutionSubMap[num].subMap[i][j].index[0] * resolution;
                    index[1] = convolutionSubMap[num].subMap[i][j].index[1] * resolution;
                    convolutionSubMap[num].position[0] = i;
                    convolutionSubMap[num].position[1] = j;
                    radius = sqrt(pow((index[0] - sharpnessMaxValueIndex[0]* resolution), 2) + pow((index[1] - sharpnessMaxValueIndex[1]* resolution), 2));
                    return radius;
                }
                else{
                    continue;
                }
            }
            
        }

        return 0;
    }

    vector<int> SecondLayer::neighbourhood(int raw, int col, int h_raw, int h_col, int step)
    {
        vector<int> result(4,0);
        result[0] = max(h_raw - step, 0);
        result[1] = min(h_raw + step, raw - 1);
        result[2] = max(h_col - step, 0);
        result[3] = min(h_col + step, col - 1);
        return result;
    }

    float SecondLayer::FindMin(float input_1,float input_2, float input_3,float input_4)
    {
        float arr[4] = {input_1,input_2,input_3,input_4};
        float output = input_1;
        for (int i = 0; i < 4; i++)
        {
            if ((arr[i] != 0 && output >= arr[i]) || output == 0)
            {
                output = arr[i];
            }
        }
        return output;
    }

    void SecondLayer::Steepness(int num)
    {
        int heightIndex[2];
        int step = convolutionSubMap[num].radius / resolution;
        vector<int> tmp;
        float up, down, left, right;
       
        tmp = neighbourhood(convolutionSubMap[num].subMap.size(), convolutionSubMap[num].subMap[0].size(), convolutionSubMap[num].position[0], convolutionSubMap[num].position[1], step);
        up = convolutionSubMap[num].subMap[tmp[0]][convolutionSubMap[num].position[1]].height;
        left = convolutionSubMap[num].subMap[convolutionSubMap[num].position[0]][tmp[2]].height;
        down = convolutionSubMap[num].subMap[tmp[1]][convolutionSubMap[num].position[1]].height;
        right = convolutionSubMap[num].subMap[convolutionSubMap[num].position[0]][tmp[3]].height;
         
         double MinHeight = FindMin(up,left,down,right);
        convolutionSubMap[num].SN = (convolutionSubMap[num].height - MinHeight) * convolutionSubMap[num].radius * 3.14;
        convolutionSubMap[num].diffHeight =  (convolutionSubMap[num].height - MinHeight);

        return;
    }

    bool SecondLayer::SharpnessIndex()
    {
        Convolution();

        vector<int> label = CCL.GetAllLabelInMap();

        convolutionSubMap.resize(label.size());

        UpdateNewConvolutionMap();

        vector<Bumps> tmp;

        for (int i = 0; i < label.size(); i++)
        {
            float height;
            double x, y;

            convolutionSubMap[i].label = label[i];
            ConvolutionSubMap(label[i], i);

            convolutionSubMap[i].radius = SharpBumpsRadius(label[i], i);
            if(convolutionSubMap[i].radius>0 && convolutionSubMap[i].radius <=sharpnessRadiusMargin)
            {
                numberOfSharpBumps += 1;
                bumpsLabel.push_back(convolutionSubMap[i].label); // Get the label value of the sharp bump
            }
        }

        if(!bumpsLabel.empty())
        {
            for (int i = 0; i < bumpsLabel.size(); i++)
            {
                for (int j = 0; j < convolutionSubMap.size(); j++)
                {
                    if(convolutionSubMap[j].label == bumpsLabel[i])
                    {
                        tmp.push_back(convolutionSubMap[j]);
                    }
                    else
                    {
                    continue;
                    }
                }
                
            }

            convolutionSubMap.clear();
            for (int i = 0; i < tmp.size(); i++)
            {
                
                convolutionSubMap.push_back(tmp[i]);

            }
            return true;
        }
        else
        {
            return false;
        }
    }

    void SecondLayer::AreaOfSharpBumps(int num)
    {
        int number = 0;
        for (int i = 0; i < convolutionSubMap[num].subMap.size(); i++)
        {
            for (int j = 0; j < convolutionSubMap[num].subMap[i].size(); j++)
            {
                if(convolutionSubMap[num].subMap[i][j].label == convolutionSubMap[num].label)
                {
                    number += 1;
                }
            }
            
        }
        convolutionSubMap[num].area = number * resolution* resolution;
    }

    double SecondLayer::DensityOfBumps(int num, int beginIndex)
    {
        AreaOfSharpBumps(num);
        int endIndex = beginIndex + areaInPlane;
        if(endIndex>=convolution_map_[0].size())
        {
            endIndex = convolution_map_[0].size();
        }
        if (convolutionSubMap[num].idex[1] >= beginIndex && convolutionSubMap[num].idex[1] <= (endIndex))
        {
            double result = convolutionSubMap[num].area / ((Map.size() * areaInPlane) * resolution * resolution);
            return result;
        }
        else
        {
            return 0;
        }
    }

    double SecondLayer::AverageSN(int size, int beginIndex)
    {
        int endIndex = beginIndex + areaInPlane;
        double area = 0.0, sn = 0.0;
        double averageSN = 0.0;
        vector<int> num;
        if (endIndex >= convolution_map_[0].size())
        {
            endIndex = convolution_map_[0].size();
        }
        for (int i = 0; i < size; i++)
        {
            AreaOfSharpBumps(i);
            if (convolutionSubMap[i].idex[1] >= beginIndex && convolutionSubMap[i].idex[1] <= (endIndex))
            {
                num.push_back(i);
            }
            else
            {
                continue;
            }
        }
        if(!num.empty())
        {
            for (int i = 0; i < num.size(); i++)
            {
                Steepness(i);
                 sn += (convolutionSubMap[i].SN/convolutionSubMap[i].area)*(convolutionSubMap[i].diffHeight/0.145);
            }
            averageSN = sn /num.size();

        }
        return averageSN;
    }

    double SecondLayer::AllDesity(int size, int beginIndex)
    {
        double density = 0;
        for (int i = 0; i < size; i++)
        {
            density += DensityOfBumps(i, beginIndex);
        }
        return density;
    }

    double SecondLayer::forwardOnce(double input)
    {
        double output;
        if (input >= 0 && input <= SNmargin_1)
        {
            output = 0;
        }
        else if(input > SNmargin_1 && input <= SNmargin_2)
        {
            output = 0.5;
        }
        else if(input > SNmargin_2 && input <= SNmargin_3)
        {
            output = 0.8;
        }
        else
        {
            output = 1;
        }
        return output;
    }


    void SecondLayer::Sa()
    {
        double snResult,densityresult;
        if(SharpnessIndex())
        {
            double averageSn = 0;
            if(convolution_map_[0].size()<=areaInPlane)
            {
                areaInPlane=convolution_map_[0].size();
                averageSn = AverageSN(convolutionSubMap.size(), 0);
                double tmp = forwardOnce(averageSn);
                PlaneSaOrigin.push_back(tmp);
                PlaneSa.push_back(averageSn);
            }
            else
            {
                for (int i = 0; i < (convolution_map_[0].size()-areaInPlane); i++)
                {
                    snResult = AverageSN(convolutionSubMap.size(), i);        
                    double tmp =forwardOnce(snResult);
                    PlaneSaOrigin.push_back(tmp);
                    PlaneSa.push_back(snResult);
                }
            }
        }
        else
        {
            vector<int> label = CCL.GetAllLabelInMap();
            if(convolution_map_[0].size()<=areaInPlane)
            {
                areaInPlane=convolution_map_[0].size();
                PlaneSaOrigin.push_back(0);
                PlaneSa.push_back(0);
            }
            else
            {
                for (int i = 0; i < (convolution_map_[0].size()-areaInPlane); i++)
                {
                    PlaneSaOrigin.push_back(0);
                    PlaneSa.push_back(0);
                }
            }
        }
        return;
    }

    void SecondLayer::CreateWheelPlane()
    {
        vector<float> tmp(areaInPlane, 0);
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            wheelPlane.push_back(tmp);
        }
        for (int i = 0; i < wheelPlane.size(); i++)
        {
            for (int j = 0; j < wheelPlane[i].size(); j++)
            {
                wheelPlane[i][j] = Function(areaInPlane, j);
            }
        }
        
    }

    double SecondLayer::Function(int size, int y)
    {
        double result = 0;
        float h = (size*resolution) / 2;
        result = pow(((y * resolution) - h), 2) + Obstacle.height;
        return result;
    }

    vector<vector<float>> SecondLayer::DiffHeight(int beginIndex)
    {
        vector<vector<float>> diffmap;
        vector<float> tmp(areaInPlane, -1);
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            diffmap.push_back(tmp);
        }
        for (int i = 0; i < wheelPlane.size(); i++)
        {
            for (int j = 0; j < wheelPlane[i].size(); j++)
            {
                if(Obstacle.subMap[i][beginIndex + j].height != 0)
                {
                    double result = wheelPlane[i][j] - Obstacle.subMap[i][beginIndex + j].height;
                    diffmap[i][j] = result;
                }
                else
                {
                    continue;
                }
            }
        }
        return diffmap;
    }

    double SecondLayer::ComputePlaneLevel(int beginIndex)
    {
        vector<vector<float>> diffmap = DiffHeight(beginIndex);
        float h_max = MaxDiff(diffmap);
        float h_min = MinDiff(diffmap);
        double result = h_min / h_max;
        return result;
    }

    double SecondLayer::ComputePlaneLevel2(int beginIndex)
    {
        vector<vector<float>> diffmap = DiffHeight(beginIndex);
        float h_max = MaxDiff(diffmap);
        float h_min = MinDiff(diffmap);
        int n_1 = 0, n_2 = 0, n_3 = 0, n_4 = 0;
        double result = 0;
        for (int i = 0; i < diffmap.size(); i++)
        {
            for (int j = 0; j < diffmap[i].size(); j++)
            {
                if(diffmap[i][j]>=0&&diffmap[i][j]<(h_max/4))
                {
                    n_1 += 1;
                }
                else if(diffmap[i][j]>=(h_max/4) && diffmap[i][j]<(h_max/2))
                {
                    n_2 += 1;
                }
                else if(diffmap[i][j]>=(h_max/2) && diffmap[i][j]<(h_max*3/4))
                {
                    n_3 += 1;
                }
                else if(diffmap[i][j]>=(h_max*3/4) && diffmap[i][j]<h_max)
                {
                    n_4 += 1;
                }
            }
            
        }   
        result = (planeWight_1 * n_1 + planeWight_2 * n_2 + planeWight_3 * n_3 + planeWight_4 * n_4) / (n_1 + n_2 + n_3 + n_4);

        return result;
    }

    float SecondLayer::MaxDiff(vector<vector<float>> diffmap)
    {
        double max = 0;
        for (int i = 0; i < diffmap.size(); i++)
        {
            for (int j = 0; j < diffmap[i].size(); j++)
            {
                if(max < diffmap[i][j])
                {
                    max = diffmap[i][j];
                }
            }
            
        }
        return max;
    }

    float SecondLayer::MinDiff(vector<vector<float>> diffmap)
    {
        float min = 10;
        for (int i = 0; i < diffmap.size(); i++)
        {
            for (int j = 0; j < diffmap[i].size(); j++)
            {
                if(min > diffmap[i][j] && diffmap[i][j] >=0)
                {
                    min = diffmap[i][j];
                }
                else
                {
                    continue;
                }
            }
            
        }
        return min;
    }

    void SecondLayer::Plane()
    {
        CreateWheelPlane();
        if(Obstacle.subMap[0].size()<=areaInPlane)
        {
            areaInPlane = Obstacle.subMap[0].size();
            double tmp = ComputePlaneLevel2(0);
            PlaneLevel.push_back(tmp);
        }
        else
        {
            for (int i = 0; i < (Obstacle.subMap[0].size()-areaInPlane); i++)
            {
                double tmp = ComputePlaneLevel2(i);
                PlaneLevel.push_back(tmp);
            }
        }
        return;

    }

    bool SecondLayer::gFittingPlane(vector<double> x, vector<double> y, vector<double> z, int n, double& a, double& b, double& c)
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

        Matrix3d m1 = Matrix3d::Random(3, 3);
        Matrix3d m2 = Matrix3d::Random(3, 3);
        Matrix3d m3 = Matrix3d::Random(3, 3);
        Matrix3d m = Matrix3d::Random(3, 3);

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
        if (!r) return false;

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

    vector<double> SecondLayer::FittingLevel(int beginindex)
    {
        vector<double> x, y, z;
        vector<double> result;
        double tmp;
        double a, b, c;
        double anglePlaneOut, anglePlaneIn;
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            for (int j = 0; j < areaInPlane; j++)
            {
                if(Obstacle.subMap[i][j].label != 0)
                {
                    x.push_back(Obstacle.subMap[i][j+beginindex].position[0]);
                    y.push_back(Obstacle.subMap[i][j+beginindex].position[1]);
                    tmp = Obstacle.subMap[i][j+beginindex].height;
                    z.push_back(tmp);
                }
            }
            
        }

        int n_1 = x.size() / 3;
        if (gFittingPlane(x, y, z, n_1, a, b, c))
        {
            double resultIN = 1.0 / (sqrt(pow(a, 2) + pow(b, 2) + 1));
            anglePlaneOut = acos((resultIN));
        }

        int n_2 = x.size();
        if (gFittingPlane(x, y, z, n_2, a, b, c))
        {
            double resultOUT = 1.0 / (sqrt(pow(a, 2) + pow(b, 2) + 1));
            anglePlaneIn = acos(resultOUT);
        }

        result.push_back(anglePlaneIn);
        result.push_back(anglePlaneOut);
        return result;
    }

    void SecondLayer::Fitting()
    {
        vector<double> result;
        if (Obstacle.subMap[0].size() <= areaInPlane)
        {
            areaInPlane = Obstacle.subMap[0].size();
            result = FittingLevel(0);
            angleIn.push_back(result[0]);
            angleOut.push_back(result[1]);
        }
        else
        {
            for (int i = 0; i < (Obstacle.subMap[0].size()-areaInPlane); i++)
            {
                result = FittingLevel(i);
                angleIn.push_back(result[0]);
                angleOut.push_back(result[1]);
            }
        }
        return;
    }

    void SecondLayer::SurfaceShape()
    {
        Sa();
        Plane();
        Fitting();
        GRA A;
        A.SetTargetSequence(targetSequence);
        A.SetWeight(graweight);
        vector<vector<double>> targetIndex;
        double num = 0;
        
        angleInOrigin.assign(angleIn.begin(), angleIn.end());
        angleOutOrigin.assign(angleOut.begin(), angleOut.end());

        targetIndex.push_back(PlaneSaOrigin);
        targetIndex.push_back(PlaneLevel);
        targetIndex.push_back(angleIn);
        targetIndex.push_back(angleOut);
       
        A.ForwardProcessing(targetIndex[0], 0);
        A.MinimumDimensionless(targetIndex[2]);
        A.MinimumDimensionless(targetIndex[3]);
        A.SetIndexSequence(targetIndex);
        A.GreyRelationCompute();
        relation = A.GetGreyRelation();
    }

    string SecondLayer::SecondLayers(string ID)
    {
        string modeID;
        if (ID == "DWLM")
        {
            ROS_INFO("Second OKK0");
            SurfaceShape();
            ROS_INFO("Second OKK1");
            UpdateMovementPath();
            ROS_INFO("Second OKK2");

            vector<PATH> maxPath;
            maxPath = RelationValueMax(movementPath);
            if (maxPath[0].relation >= graMargin)
            {
                modeID = "DWLM";
                saResult.push_back(PlaneSa[maxPath[0].index]);
                saResult.push_back(PlaneSa[maxPath[1].index]);
                planeLevelResult.push_back(PlaneLevel[maxPath[0].index]);
                planeLevelResult.push_back(PlaneLevel[maxPath[1].index]);
                angleInResult.push_back(angleInOrigin[maxPath[0].index]);
                angleInResult.push_back(angleInOrigin[maxPath[1].index]);
                angleOutResult.push_back(angleOutOrigin[maxPath[0].index]);
                angleOutResult.push_back(angleOutOrigin[maxPath[1].index]);
                relationResult.push_back(maxPath[0].relation);
                relationResult.push_back(maxPath[1].relation);
            }
            else
            {
                modeID = "WCM";
                saResult.push_back(AverageValue(PlaneSa));
                planeLevelResult.push_back(AverageValue(PlaneLevel));
                angleInResult.push_back(AverageValue(angleInOrigin));
                angleOutResult.push_back(AverageValue(angleOutOrigin));
                relationResult.push_back(AverageValue(relation));
            }

            return modeID;
        }
        else{
            ROS_INFO("Second OKK0");
            SurfaceShape();
            ROS_INFO("Second OKK1");
            UpdateMovementPath();
            ROS_INFO("Second OKK2");
            saResult.push_back(AverageValue(PlaneSa));
            planeLevelResult.push_back(AverageValue(PlaneLevel));
            angleInResult.push_back(AverageValue(angleInOrigin));
            angleOutResult.push_back(AverageValue(angleOutOrigin));
            relationResult.push_back(AverageValue(relation));
            return ID;
        }
    }


    double SecondLayer::AverageValue(vector<double> input)
    {
        double result = 0;
        for (int i = 0; i < input.size(); i++)
        {
            result += input[i];
        }
        result = result / input.size();
        return result;
    }

    void SecondLayer::UpdateMovementPath()
    {
        PATH tmp;
        vector<double> temp(2, 0);
        for (int i = 0; i < relation.size(); i++)
        {
            tmp.relation = relation[i];
            tmp.index = i;
            tmp.distance = temp;
            movementPath.push_back(tmp);
        }
    }

    void SecondLayer::SetRobotPath(vector<vector<double>> position)
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

    void SecondLayer::SetWheelPath()
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

    vector<double> SecondLayer::PathDistance(int num)
    {
        double positionUP, positionDOWN;
        for (int i = 0; i < Obstacle.subMap.size(); i++)
        {
            for (int j = 0; j < Obstacle.subMap[i].size(); j++)
            {
                if(Obstacle.subMap[i][j].label==Obstacle.label)
                {
                    positionUP = Obstacle.subMap[i][j].position[0];
                }
            }
            
        }
        for (int i = Obstacle.subMap.size() - 1; i >= 0; i++)
        {
            for (int j = 0; j < Obstacle.subMap[i].size(); j++)
            {
                if(Obstacle.subMap[i][j].label==Obstacle.label)
                {
                    positionDOWN = Obstacle.subMap[i][j].position[0];
                }
            }
            
        }

        double y_right, y_left;
        for (int i = 0; i < rightWheelPath.size(); i++)
        {
            if(rightWheelPath[i][0]>=(positionDOWN-0.01)&&rightWheelPath[i][0]<=(positionDOWN+0.01))
            {
                y_right=rightWheelPath[i][1];
                y_left=leftWheelPath[i][1];
            }
        }

        vector<double> pathDistance;
            for (int i = 0; i < Obstacle.subMap.size(); i++)
            {
                if(Obstacle.subMap[i][num].label==Obstacle.label)
                {
                    double re_1, re_2;
                    re_1 = Obstacle.subMap[i][num].position[1] - y_right;
                    re_2 = Obstacle.subMap[i][num].position[1] - y_left;
                    pathDistance.push_back(re_2);
                    pathDistance.push_back(re_1);
                    break;
                }
            }
            
        return pathDistance;
    }

    vector<PATH> SecondLayer::RelationValueMax(vector<PATH> input)
    {
        
        vector<PATH> wheelPath;
        PATH wheel_1, wheel_2;
        int wheelDistance = (robotWide - 0.2) / resolution;
        int leftIndex = input.size() - wheelDistance;
        if ( input.size() > wheelDistance )
        {
            double tmp = input[0].relation + input[wheelDistance].relation;
            wheel_1.relation = input[0].relation;
            wheel_1.index = input[0].index;
            wheel_2.relation = input[wheelDistance].relation;
            wheel_2.index = input[wheelDistance].index;
            for (int i = 0; i < leftIndex; i++)
            {
                if ( input[i].relation >= graMargin && input[i + wheelDistance].relation >= graMargin)
                {
                    if ( tmp < (input[i].relation + input[i+wheelDistance].relation))
                    {
                        tmp = input[i].relation + input[i + wheelDistance].relation;
                        wheel_1.relation = input[i].relation;
                        wheel_1.index = input[i].index;
                        wheel_2.relation = input[i + wheelDistance].relation;
                        wheel_2.index = input[i + wheelDistance].index;
                    }
                }
                else
                {
                    continue;
                }
                
            }
            wheelPath.push_back(wheel_1);
            wheelPath.push_back(wheel_2);
            return wheelPath;
        }
        else
        {
            PATH Max;
            Max.relation = 0;
            for (int i = 0; i < input.size(); i++)
            {
                if(Max.relation <= input[i].relation)
                {
                    Max.relation = input[i].relation;
                    Max.index = input[i].index;
                    Max.distance = input[i].distance;
                }
            }
            wheelPath.push_back(Max);
            wheelPath.push_back(Max);
            return wheelPath;
        }
    }

    vector<double> SecondLayer::GetSaInObstacle()
    {
        return PlaneSa;
    }

    vector<double> SecondLayer::GetPlaneLevelInObstacle()
    {
        return PlaneLevel;
    }

    vector<double> SecondLayer::GetAngleInObstacle()
    {
        return angleInOrigin;
    }

    vector<double> SecondLayer::GetGreyRelationValue()
    {
        return relation;
    }

    vector<double> SecondLayer::GetAngleOutObstacle()
    {
        return angleOutOrigin;
    }

    vector<double> SecondLayer::GetSaResult()
    {
        return saResult;
    }

    vector<double> SecondLayer::GetPlaneLevelResult()
    {
        return planeLevelResult;
    }

    vector<double> SecondLayer::GetAngleInResult()
    {
        return angleInResult;
    }

    vector<double> SecondLayer::GetAngleOutResult()
    {
        return angleOutResult;
    }

    vector<double> SecondLayer::GetRelationResult()
    {
        return relationResult;
    }

} // namespace SecondLayer
