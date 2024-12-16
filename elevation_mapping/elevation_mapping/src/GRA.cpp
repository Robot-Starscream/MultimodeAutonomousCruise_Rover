/*
 * GRA.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "ros/ros.h"
#include "elevation_mapping/GRA.h"
#include <cmath>

using namespace std;

namespace Grey
{

	GRA::GRA()
	{
		row = 0.5;	
	}

	void GRA::SetTargetSequence(vector<double> target_)
	{
		targetSequence = target_;
	}

	void GRA::SetIndexSequence(vector<vector<double>> index_)
	{
		indexSequence = index_;
	}

	void GRA::SetWeight(vector<double> w)
	{
		weight = w;
	}

	double GRA::PolarityMax(vector<double> targetIndex_)
	{
		double tmp = fabs(targetSequence[0] - targetIndex_[0]);
		for (int i = 0; i < targetIndex_.size(); i++)
		{
			if (tmp < (fabs(targetSequence[i] - targetIndex_[i])))
			{
				tmp = fabs(targetSequence[i] - targetIndex_[i]);
			}
		}
		return tmp;

	}

	double GRA::PolarityMin(vector<double> targetIndex_)
	{
		double tmp = fabs(targetSequence[0] - targetIndex_[0]);
		for (int i = 0; i < targetIndex_.size(); i++)
		{
			if (tmp > (fabs(targetSequence[i] - targetIndex_[i])))
			{
				tmp = fabs(targetSequence[i] - targetIndex_[i]);
			}
		}
		return tmp;

	}

	double GRA::IndexMax(vector<double> targetIndex_)
	{
		double tmp = targetIndex_[0];
		for (int i = 0; i < targetIndex_.size(); i++)
		{
			if (tmp < targetIndex_[i])
			{
				tmp = targetIndex_[i];
			}
			else
			{
				continue;
			}
		}
		return tmp;
	}

	double GRA::IndexMin(vector<double> targetIndex_)
	{
		double tmp = targetIndex_[0];
		for (int i = 0; i < targetIndex_.size(); i++)
		{
			if (tmp > targetIndex_[i])
			{
				tmp = targetIndex_[i];
			}
			else
			{
				continue;
			}
		}
		return tmp;
	}

	void GRA::ForwardProcessing(vector<double> &index_,int num)
	{
		double Max = IndexMax(index_);
		for (int i = 0; i < index_.size(); i++)
		{
			index_[i] = fabs(targetSequence[num] - index_[i]);
		}
		return;
	}

	void GRA::MaximumDimensionless(vector<double>& index_)
	{
		double MAX = IndexMax(index_);
		double MIN = IndexMin(index_);
		for (int i = 0; i < index_.size(); i++)
		{
			index_[i] = (index_[i] - MIN) / (MAX - MIN);
		}
		return;
	}


	void GRA::MinimumDimensionless(vector<double>& index_)
	{
		double MAX = IndexMax(index_);
		double MIN = IndexMin(index_);
		for (int i = 0; i < index_.size(); i++)
		{
			index_[i] = (MAX - index_[i]) / (MAX - MIN);
		}
		return;
	}

	void GRA::PushIndex(vector<double> index_)
	{
		indexSequence.push_back(index_);
	}

	vector<vector<double>> GRA::TransPose(vector<vector<double>> index_)
	{
		vector<vector<double>> result;
		for (int j = 0; j < index_[0].size(); j++)
		{
			vector<double> tmp;
			for (int i = 0; i < index_.size(); i++)
			{
				tmp.push_back(index_[i][j]);
			}
			result.push_back(tmp);
		}
		return result;
	}

	void GRA::GreyRelationCompute()
	{
		vector<vector<double>> Index = TransPose(indexSequence);//indexSequence 4x7，index 7x4
		vector<double> tmp_1, tmp_2;
		double memberReslution_1, memberReslution_2;
		for (int i = 0; i < Index.size(); i++)
		{
			double member_1 = PolarityMin(Index[i]);
			
			double member_2 = PolarityMax(Index[i]);
			
			tmp_1.push_back(member_1);
			tmp_2.push_back(member_2);
		}
		memberReslution_1 = IndexMin(tmp_1);
		memberReslution_2 = IndexMax(tmp_2);
		for (int i = 0; i < Index.size(); i++)
		{
			vector<double> re;
			/*re.push_back(i);*/
			for (int j = 0; j < Index[i].size(); j++)
			{
				re.push_back((memberReslution_1 + row * memberReslution_2) / (fabs(targetSequence[j] - Index[i][j]) + (row * memberReslution_2)));
			}
			relationMatrix.push_back(re);
		}
		double tmp = 0;
		for (int i = 0; i < relationMatrix.size(); i++)
		{
			for (int j = 0; j < relationMatrix[i].size(); j++)
			{
				
				tmp += weight[j] * relationMatrix[i][j];
			}
			tmp = tmp / weight.size();
			relation.push_back(tmp);
			tmp = 0;
		}
	}

	vector<double> GRA::GetGreyRelation()
	{
		return relation;
	}
	vector<vector<double>> GRA::GetRelationMatrix()
	{
		return relationMatrix;
	}
} // namespace GRA


