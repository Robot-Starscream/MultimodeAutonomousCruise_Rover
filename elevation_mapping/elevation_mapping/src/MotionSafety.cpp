/*
 * MotionSafety.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "elevation_mapping/MotionSafety.h"

using namespace std;

namespace MotionSafety
{
    // Fuzzy Rule Base Initialization
    MotionSafety::MotionSafety()
    {
        terrainRoughness.deFuzzyValue = 0;
        terrainSlope.deFuzzyValue = 0;
        complexity.deFuzzyValue = 0;
        wheelAdaptability.deFuzzyValue = 0;
        robotTrackSmoothness.deFuzzyValue = 0;
        adventureDegree.deFuzzyValue = 0;

        vector<vector<double>> memOfLateralF = DesignMembershipFuction(0, 81.0, 3, 0.2);
        vector<vector<double>> memOfRollAng = DesignMembershipFuction(0, 15, 3, 0.2);
        vector<vector<double>> memOfRoughTerr = DesignMembershipFuction(0, 6.0, 3, 0.2);//

        double normal[4] = { 0, 400, 800, 1200 };
        vector<double> vecb(4, 0);
        vector<vector<double>> memOfNormalF(3, vector<double>(4, 0));//
        VectorPushBack(vecb, normal);
        memOfNormalF = DesignMembershipFuction(vecb, 0.2);
        double pitch[4] = { -30, -20, 20, 30 };
        vector<vector<double>> memOfPitchAng(3, vector<double>(4, 0));//
        VectorPushBack(vecb, pitch);
        memOfPitchAng = DesignMembershipFuction(vecb, 0.2);
        vector<vector<double>> memOfSlopeTerr = DesignMembershipFuction(-3.0, 3.0, 5, 0.2);//

        vector<vector<double>> memOfComplexity = DesignMembershipFuction(0, 6.0, 3, 0.2);//

        double maxNormal[4] = { 0, 200, 1160, 1500 };
        vector<vector<double>> memOfMaxDiffNormalForce(3, vector<double>(4, 0));//
        VectorPushBack(vecb, maxNormal);
        memOfMaxDiffNormalForce = DesignMembershipFuction(vecb, 0.2);
        double averageNormal[4] = { 0, 100, 600, 900 };
        vector<vector<double>> memOfAverageNormalForce(3, vector<double>(4, 0));//
        VectorPushBack(vecb, averageNormal);
        memOfAverageNormalForce = DesignMembershipFuction(vecb, 0.2);
        vector<vector<double>> memOfWheelAdaptability = DesignMembershipFuction(0, 6.0, 3, 0.2);//

        double addLateral[4] = { 0, 1200, 1600, 2000 };
        vector<vector<double>> memOfAddLateralAcc(3, vector<double>(4, 0));//
        VectorPushBack(vecb, addLateral);
        memOfAddLateralAcc = DesignMembershipFuction(vecb, 0.2);
        double addVertical[4] = { 0, 200, 600, 800 };
        vector<vector<double>> memOfAddVerticalAcc(3, vector<double>(4, 0));//
        VectorPushBack(vecb, addVertical);
        memOfAddVerticalAcc = DesignMembershipFuction(vecb, 0.2);
        vector<vector<double>> memOfRobotTrackSmoothness = DesignMembershipFuction(0, 6.0, 3, 0.2);//

        vector<vector<double>> memOfAdventureDegree = DesignMembershipFuction(0, 6.0, 5, 0.2);//

        string tmps10[3] = { "Small", "Medium", "Large" };
        vector<string> smallStr3(tmps10, tmps10 + 3);
        AddLinguisticValue(lateralForce, memOfLateralF, smallStr3);
        AddLinguisticValue(normalForce, memOfNormalF, smallStr3);
        AddLinguisticValue(maxDiffNormalForce, memOfMaxDiffNormalForce, smallStr3);
        AddLinguisticValue(averageNormalForce, memOfAverageNormalForce, smallStr3);
        AddLinguisticValue(additionalLateralAcc, memOfAddLateralAcc, smallStr3);

        string tmps20[3] = { "Low", "Medium", "High" };
        vector<string> lowStr3(tmps20, tmps20 + 3);
        AddLinguisticValue(rollAngle, memOfRollAng, lowStr3);
        AddLinguisticValue(pitchAngle, memOfPitchAng, lowStr3);
        AddLinguisticValue(complexity, memOfComplexity, lowStr3);
        AddLinguisticValue(additionalVerticalAcc, memOfAddVerticalAcc, lowStr3);

        string tmps30[3] = { "Smooth", "Rough", "Bumpy" };
        vector<string> roughStr3(tmps30, tmps30 + 3);	
        AddLinguisticValue(terrainRoughness, memOfRoughTerr, roughStr3);

        string tmps40[5] = { "DownHill", "DownSlope", "Flat", "UpSlope", "UpHill" };
        vector<string> slopeStr5(tmps40, tmps40 + 5);
        AddLinguisticValue(terrainSlope, memOfSlopeTerr, slopeStr5);
        
        string tmps50[3] = { "Weak", "Medium", "Strong" };
        vector<string> weakStr3(tmps50, tmps50 + 3);
        AddLinguisticValue(wheelAdaptability, memOfWheelAdaptability, weakStr3);

        string tmps60[3] = { "Uneven", "Medium", "Even" };
        vector<string> evenStr3(tmps60, tmps60 + 3);
        AddLinguisticValue(robotTrackSmoothness, memOfRobotTrackSmoothness, evenStr3);

        string tmps70[5] = { "I", "II", "III", "IV", "V" };
        vector<string> degreeStr5(tmps70, tmps70 + 5);
        AddLinguisticValue(adventureDegree, memOfAdventureDegree, degreeStr5);

        //Fuzzy Rules For Roughness
        roughnessRules.resize(9);
        string tmps80[9] = { roughStr3[0], "Rough", "Bumpy", "Rough", "Rough", "Bumpy", "Bumpy", "Bumpy", "Bumpy" }; 
        vector<string> result_list_roughness(tmps80, tmps80 + 9);
        AddTokensToRule(roughnessRules, &lateralForce, &rollAngle, &terrainRoughness, result_list_roughness);

        //Fuzzy Rules For Slope
        slopeRules.resize(9);
        string tmps90[9] = { "Flat", "UpSlope", "UpHill", "DownSlope", "Flat", "UpSlope", "DownHill", "DownSlope", "Flat" };
        vector<string> result_list_slope(tmps90, tmps90 + 9);
        AddTokensToRule(slopeRules, &normalForce, &pitchAngle, &terrainSlope, result_list_slope);

        //Fuzzy Rules For contact-surface complexity
        complexityRules.resize(15);
        string tmps100[15] = { "Medium", "High", "High", "Medium", "Medium", "High", "Low", "Medium", "Medium", "Medium", "Medium", "High", "Medium",  "High", "High" };
        vector<string> result_list_complexity(tmps100, tmps100 + 15);
        AddTokensToRule(complexityRules, &terrainSlope, &terrainRoughness, &complexity, result_list_complexity);

        //Fuzzy Rules For Wheel Adaptability
        adaptabilityRules.resize(9);
        string tmps110[9] = { "Strong", "Medium", "Weak", "Medium", "Medium", "Weak", "Weak", "Weak", "Weak" };
        vector<string> result_list_adaptability(tmps110, tmps110 + 9);
        AddTokensToRule(adaptabilityRules, &maxDiffNormalForce, &averageNormalForce, &wheelAdaptability, result_list_adaptability);

        //Fuzzy Rules For robot track smoothness
        smoothnessRules.resize(9);
        string tmps120[9] = { "Even", "Medium", "Uneven", "Medium", "Medium", "Uneven", "Uneven", "Uneven", "Uneven" };
        vector<string> result_list_smoothness(tmps120, tmps120 + 9);
        AddTokensToRule(smoothnessRules, &additionalLateralAcc, &additionalVerticalAcc, &robotTrackSmoothness, result_list_smoothness);

        //Fuzzy Rules For Adventure Degree by Wheel Adaptability
        adventureRules_WheelAdap.resize(9);
        string tmps130[9] = { "III", "I", "I", "IV", "II", "I", "V", "IV", "III" };
        vector<string> result_list_adventure(tmps130, tmps130 + 9);
        AddTokensToRule(adventureRules_WheelAdap, &complexity, &wheelAdaptability, &adventureDegree, result_list_adventure);

        //Fuzzy Rules For Adventure Degree by Robot Track Smoothness
        adventureRules_RobTrackSmooth.resize(9);
        AddTokensToRule(adventureRules_RobTrackSmooth, &complexity, &robotTrackSmoothness, &adventureDegree, result_list_adventure);

    }

    void MotionSafety::ResetAll()
    {
        ResetLinguisticVariable(&terrainRoughness);
        ResetLinguisticVariable(&terrainSlope);
        ResetLinguisticVariable(&wheelBase);
        ResetLinguisticVariable(&robotMobility);
        ResetLinguisticVariable(&steeringMovement);
    }

    void MotionSafety::ResetLinguisticVariable(LinguisticVariable* p_linguisticVar)
    {
        unsigned int size = p_linguisticVar->linguisticValueVector.size();
        for (unsigned int i = 0; i < size; i++) {
            p_linguisticVar->linguisticValueVector[i].linguisticValue = 0;
        }
    }

    vector<vector<double>> MotionSafety::DesignMembershipFuction(const vector<double>& vec, double upRate) 
    {
        // Membership function non-uniformly distributed

        int setNum = vec.size() - 1;
        vector<vector<double>> result(setNum, vector<double>(4, 0));
        double leftBorder = vec[0];
        double rightBorder = vec[setNum];
        vector<double> setLength(setNum);
        //for (int i = 0; i < setNum; i++) {
        //	setLength[i] = vec[i + 1] - vec[i];
        //}
        result[0][0] = leftBorder;
        result[0][1] = leftBorder;
        result[0][2] = leftBorder + (vec[1] - vec[0]) * upRate;
        result[0][3] = leftBorder + (vec[1] - vec[0]);

        result[setNum - 1][0] = rightBorder - (vec[setNum] - vec[setNum - 1]);
        result[setNum - 1][1] = rightBorder - (vec[setNum] - vec[setNum - 1]) * upRate;
        result[setNum - 1][2] = rightBorder;
        result[setNum - 1][3] = rightBorder;
        for (int i = 1; i < setNum - 1; i++) {
            result[i][0] =  1.5 * vec[i] - 0.5 * vec[i + 1]; 
            result[i][1] = (0.5 + upRate) * vec[i] + (0.5 - upRate) * vec[i + 1];   
            result[i][2] = (0.5 - upRate) * vec[i] + (0.5 + upRate) * vec[i + 1];
            result[i][3] = 1.5 * vec[i + 1] - 0.5 * vec[i]; 
        }

        return result;
    }

    vector<vector<double>> MotionSafety::DesignMembershipFuction(double leftBorder, double rightBorder, int setNum, double upRate)
    {
        // Membership function uniformly distributed
        //WARNING: setNum must >= 3, else the program will crash
        //setNum means the nums of linguistic set; upRate means the half rate of upper base of trapezoid in whole setLength; hypRate means the rate of hypotenuse of trapezoid in whole setLength
        //Basic constrains for upRate and hypRate: 1) 0 < upRate < 0.5; 2) 0 < hypRate < 1; 3) 0.5 <= upRate + hypRate <= 1.
        //The larger hypRate is, the stronger anti-disturbance ability is. 
        vector<vector<double>> result(setNum, vector<double>(4, 0));
        double setLength = (rightBorder - leftBorder) / setNum;
        vector<double> center(setNum - 2);
        for (int i = 0; i < setNum - 2; i++) {
            center[i] = (i + 1.5) * setLength;
        }
        double len1 = upRate * setLength;

        result[0][0] = leftBorder;
        result[0][1] = leftBorder;
        result[0][2] = leftBorder + len1;
        result[0][3] = leftBorder + setLength;

        result[setNum - 1][0] = rightBorder - setLength;
        result[setNum - 1][1] = rightBorder - len1;
        result[setNum - 1][2] = rightBorder;
        result[setNum - 1][3] = rightBorder;
        for (int i = 1; i < setNum - 1; i++) {
            result[i][0] = center[i - 1] - setLength;
            result[i][1] = center[i - 1] - len1;
            result[i][2] = center[i - 1] + len1;
            result[i][3] = center[i - 1] + setLength;
        }

        return result;
    }

    void MotionSafety::AddLinguisticValue(LinguisticVariable& p_linguisticVariable, vector<vector<double>>& nums, const vector<string>& valueName)
    {
        if (nums.size() == valueName.size()) {
            int size = nums.size();
            for (int i = 0; i < size; i++) {
                LinguisticValue linguisticValue = { nums[i][0],nums[i][1],nums[i][2],nums[i][3],valueName[i],0 };
                p_linguisticVariable.linguisticValueVector.emplace_back(linguisticValue);
            }
        }
    }

    void MotionSafety::AddTokenToRule(FuzzyRule& p_fuzzyRule, Operation operation, LinguisticVariable* linguisticVariable, string valueName)
    {
        FuzzyRuleToken fuzzyRuleToken = { operation,linguisticVariable ,valueName };
        p_fuzzyRule.v_fuzzyRuleTokens.emplace_back(fuzzyRuleToken);
    }

    void MotionSafety::AddTokensToRule(vector<FuzzyRule>& v_fuzzyRule, LinguisticVariable* first, LinguisticVariable* second, LinguisticVariable* result, const vector<string>& r_str)
    {
        int size = v_fuzzyRule.size();
        vector<string> f_str(first->valueName);
        vector<string> s_str(second->valueName);
        int x = s_str.size();
        int j, k = 0;
        for (int i = 0; i < size; i++) {
            j = i / x;
            k = i % x;
            AddTokenToRule(v_fuzzyRule[i], BEGIN, first, f_str[j]);
            AddTokenToRule(v_fuzzyRule[i], AND, second, s_str[k]);
            AddTokenToRule(v_fuzzyRule[i], END, result, r_str[i]);
        }

    }

    double MotionSafety::CalculateLinguisticValue(LinguisticValue* p_linguisticValue, double input)
    {
        double nA = p_linguisticValue->A;
        double nB = p_linguisticValue->B;
        double nC = p_linguisticValue->C;
        double nD = p_linguisticValue->D;

        //Calculating Trapezoidal function
        if (nA == nB) {
            if (input <= nC)
                return 1;
            else if (nC < input && input < nD)
                return (nD - input) / (nD - nC);
            else
                return 0;
        }
        else if (nC == nD) {
            if (input <= nA)
                return 0;
            else if (nA < input && input < nB)
                return (input - nA) / (nB - nA);
            else
                return 1;
        }
        else {
            if ((input <= nA) || (input >= nD))
                return 0;
            else if ((nA < input) && (input < nB))
                return (input - nA) / (nB - nA);
            else if ((nB <= input) && (input <= nC))
                return 1;
            else
                return (nD - input) / (nD - nC);
        }
    }

    void MotionSafety::UpdateLinguisticValue(LinguisticValue* p_linguisticValue, double newLinguisticValue)
    {
        p_linguisticValue->linguisticValue = OrOperator(p_linguisticValue->linguisticValue, newLinguisticValue);
    }

    LinguisticValue* MotionSafety::FindLinguisticValueByName(LinguisticVariable* p_linguisticVariable, string valueName)
    {
        unsigned int i = 0;
        unsigned int searchValueSize = p_linguisticVariable->linguisticValueVector.size();
        LinguisticValue* iter;
        do {
            iter = &(p_linguisticVariable->linguisticValueVector[i]);
            i++;
        } while (iter->s_valueName != valueName && i < searchValueSize);

        return iter;
    }

    vector<LinguisticValue*> MotionSafety::OutputLinguisticValueList(LinguisticVariable& p_linguisticVariable)
    {
        vector<LinguisticValue*> linguisticValuesList;
        unsigned int size = p_linguisticVariable.linguisticValueVector.size();
        for (unsigned int i = 0; i < size; i++) {
            LinguisticValue* tmp = &(p_linguisticVariable.linguisticValueVector[i]);
            linguisticValuesList.emplace_back(tmp);
        }

        return linguisticValuesList;
    }

    void MotionSafety::VectorPushBack(vector<vector<double>>& vec, double a[][4])
    {
        int size = vec.size();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < 4; j++) {
                vec[i][j] = a[i][j];
            }
        }
    }

    void MotionSafety::VectorPushBack(vector<double>& vec, double b[4])
    {
        int size = vec.size();
        for (int i = 0; i < size; i++) {
            vec[i] = b[i];
        }
    }

    double MotionSafety::OrOperator(double A, double B)
    {
        if (A > B)
            return A;
        return B;
    }

    double MotionSafety::AndOperator(double A, double B)
    {
        if (A > B)
            return B;
        return A;
    }

    double MotionSafety::CalculateTrapezoid(LinguisticValue* p_linguisticValue)
    {
        if (p_linguisticValue->A == p_linguisticValue->B) {
            if (p_linguisticValue->linguisticValue < 1) return (p_linguisticValue->C + p_linguisticValue->D) / 2;
        }
        if (p_linguisticValue->C == p_linguisticValue->D) {
            if (p_linguisticValue->linguisticValue < 1) return (p_linguisticValue->A + p_linguisticValue->B) / 2;
        }
        return (p_linguisticValue->B + p_linguisticValue->C) / 2;
    }

    FuzzyRuleToken* MotionSafety::ReasonFuzzyRule(FuzzyRule& v_fuzzyRule)
    {
        double resultVal = 0;
        unsigned int tokenSize = v_fuzzyRule.v_fuzzyRuleTokens.size();
        for (unsigned int i = 0; i < tokenSize; i++) {
            FuzzyRuleToken* p_token = &(v_fuzzyRule.v_fuzzyRuleTokens[i]);
            LinguisticValue* p_value = FindLinguisticValueByName(p_token->p_linguisticVariable, p_token->linguisticValueName);
            if (p_token->operation != END) {
                double tokenVal = CalculateLinguisticValue(p_value, p_token->p_linguisticVariable->deFuzzyValue);

                if (tokenVal == 0) return nullptr;
                    
                p_value->linguisticValue = tokenVal;

                if (p_token->operation == BEGIN)
                    resultVal = tokenVal;
                else if (p_token->operation == AND)
                    resultVal = AndOperator(resultVal, tokenVal);
                else
                    resultVal = OrOperator(resultVal, tokenVal);
            }
            else {
                UpdateLinguisticValue(p_value, resultVal);
                return p_token;
            }
        }
        return nullptr;
    }

    double MotionSafety::Defuzzy(LinguisticVariable& p_linguisticVar)
    {
        double upEqualation = 0;
        double downEqualation = 0;
        vector <LinguisticValue*> v_linguisticValue = OutputLinguisticValueList(p_linguisticVar);
        unsigned int valueSize = v_linguisticValue.size();

        for (unsigned int i = 0; i < valueSize; i++) {
            LinguisticValue* val = v_linguisticValue.at(i);
            upEqualation += val->linguisticValue * CalculateTrapezoid(val);
            downEqualation += val->linguisticValue;
        }
        if (downEqualation != 0)
            return p_linguisticVar.deFuzzyValue = upEqualation / downEqualation;
        else {
            return p_linguisticVar.deFuzzyValue = 0;
        }
    }

    double MotionSafety::CalculateFuzzyRules(vector<FuzzyRule> v_fuzzyRules, LinguisticVariable& p_linguisticVar)
    {
        unsigned int ruleSize = v_fuzzyRules.size();

        for (unsigned int i = 0; i < ruleSize; i++) {
            ReasonFuzzyRule(v_fuzzyRules[i]);
        }

        return Defuzzy(p_linguisticVar);
    }

    string MotionSafety::GetLinguisticValueNameAsOutput(const LinguisticVariable& p_linguisticVar)
    {
        string level;
        vector<string> levels = {"I", "II", "III", "IV", "V"};
        vector<double> numbers;
        double max_value;

        for (int i = 0; i < levels.size(); i++)
        {
            double number = FindLinguisticValueByName(&p_linguisticVar, levels[i])->linguisticValue;
            numbers.push_back(number);
        }
        
        if (!numbers.empty()) {
            max_value = *std::max_element(numbers.begin(), numbers.end());
        }

        for (int i = 0; i < numbers.size(); i++)
        {
            if (numbers[i] >= max_value) {
                level = levels[i];
                break;
            }
        }

        retrun level;
    }

    double MotionSafety::RiskEvaluation_1(InputData idata, double id, string &level)
    {
        double d_roughness, d_slope, d_complexity, d_adaptability, d_smoothness, d_adventure;

        lateralForce.deFuzzyValue = abs(idata.F_Llrw);     // N
        rollAngle.deFuzzyValue = abs(idata.roll);        // deg
        normalForce.deFuzzyValue = idata.F_Nfrw;    // N
        pitchAngle.deFuzzyValue = idata.pitch;      // deg
        maxDiffNormalForce.deFuzzyValue = idata.delta_W_max; // N
        averageNormalForce.deFuzzyValue = idata.W_fmr; // N
        additionalLateralAcc.deFuzzyValue = abs(idata.Acc_L);
        additionalVerticalAcc.deFuzzyValue = abs(idata.Acc_Z);

        ResetAll();
        d_roughness = CalculateFuzzyRules(roughnessRules, terrainRoughness);
        d_slope = CalculateFuzzyRules(slopeRules, terrainSlope);
        d_complexity = CalculateFuzzyRules(complexityRules, complexity);

        d_smoothness = CalculateFuzzyRules(smoothnessRules, robotTrackSmoothness);
		d_adventure = CalculateFuzzyRules(adventureRules_RobTrackSmooth, adventureDegree);

        level = GetLinguisticValueNameAsOutput(adventureDegree);
        if (level == "IV" || level == "V") {
            id = 5.0;
        }
        return id;
    }

    double MotionSafety::RiskEvaluation_2(InputData idata, double id, string &level)
    {
        double d_roughness, d_slope, d_complexity, d_adaptability, d_smoothness, d_adventure;

        lateralForce.deFuzzyValue = abs(idata.F_Llrw);     // N
        rollAngle.deFuzzyValue = abs(idata.roll);        // deg
        normalForce.deFuzzyValue = idata.F_Nfrw;    // N
        pitchAngle.deFuzzyValue = idata.pitch;      // deg
        maxDiffNormalForce.deFuzzyValue = idata.delta_W_max; // N
        averageNormalForce.deFuzzyValue = idata.W_fmr; // N
        additionalLateralAcc.deFuzzyValue = abs(idata.Acc_L);
        additionalVerticalAcc.deFuzzyValue = abs(idata.Acc_Z);

        ResetAll();
        d_roughness = CalculateFuzzyRules(roughnessRules, terrainRoughness);
        d_slope = CalculateFuzzyRules(slopeRules, terrainSlope);
        d_complexity = CalculateFuzzyRules(complexityRules, complexity);

        d_adaptability = CalculateFuzzyRules(adaptabilityRules, wheelAdaptability);
		d_adventure = CalculateFuzzyRules(adventureRules_WheelAdap, adventureDegree);

        level = GetLinguisticValueNameAsOutput(adventureDegree);
        if (level == "IV" || level == "V") {
            id = 5.0;
        }
        return id;
    }

} // namespace MotionSafety
