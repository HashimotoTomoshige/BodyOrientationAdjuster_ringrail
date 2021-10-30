#ifndef COMPUTE_KINEMATICS_H
#define COMPUTE_KINEMATICS_H

#include "compute_motor_angle.h"

namespace robot_posture_control
{
    class ComputeKinematics : public ComputeMotorAngle
    {
    protected:
        Eigen::VectorXd r_ = Eigen::VectorXd::Constant(3,0);
        Eigen::VectorXd gui_goal_ = Eigen::VectorXd::Constant(2,0);
        
    public:
        ComputeKinematics(); //基底クラスのコンストラクタを実行するため
        Eigen::MatrixXd computeT(const Eigen::VectorXd& q); //DH法で同次変換行列を算出する関数
        std::vector<double> vectorAddition(std::vector<double> a, std::vector<double> b);
        std::vector<double> vectorSubtraction(std::vector<double> a, std::vector<double> b);
        std::vector<double> getLinkParamFromQ2(const double& q_2);
        Eigen::MatrixXd computeJ(const Eigen::VectorXd& q);
        Eigen::VectorXd getPosture(const Eigen::VectorXd& q);
        double atan2ChangeDomain(double y, double x);
        Eigen::VectorXd computePosture(const Eigen::VectorXd& r_goal); //目標の手先位置姿勢ベクトルから関節変位ベクトルを算出する関数
        Eigen::VectorXd inputGoal();
        Eigen::VectorXd inputGoalFromGui();
        Eigen::VectorXd ConvertMotionAxisIntoPosture(const double& roll, const double& pitch);
        bool write_flag_ = false;
        double chest_angle_ = 0;


        void dataDisplay(); //モーター回転角. 同次変換行列を画面に表示
            

    };
};

#endif

