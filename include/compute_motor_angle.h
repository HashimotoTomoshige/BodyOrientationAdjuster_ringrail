#ifndef COMPUTE_MOTOR_ANGLE_H
#define COMPUTE_MOTOR_ANGLE_H

#include "include_files.h"

namespace robot_posture_control
{
    class ComputeMotorAngle
    {
    protected:
        //std::vector<double> theta_; //関節角q1, q2, q3

    public:
        ComputeMotorAngle(); //関節角を度数法から弧度法に変換するコンストラクタ
        int convertWaistAngleToRotationNumber(const double& theta); //腰部回旋機構の関節角をモーター角度に変換
        int convertBendingAngleToRotationNumber(const double& theta); //屈曲機構の関節角をモーター角度に変換
        int convertChestAngleToRotationNumber(const double& theta); //胸部回旋機構の関節角をモーター角度に変換
        int convertBaseAngleToRotationNumber(const double& theta); //土台の関節角をモーター角度に変換
        //std::vector<int> motor_angle_; //モーター角度(オリエンタルモーターRK2シリーズは1回転あたり500)
    };
};

#endif