#include "compute_kinematics.h"

namespace robot_posture_control
{
    ComputeKinematics::ComputeKinematics() : ComputeMotorAngle(){}

    double ComputeKinematics::atan2ChangeDomain(double y, double x)
    {
        double theta = atan2(y, x);
        if(theta < 0)
        {
            theta += 2 * M_PI;
        }
        return theta;
    }



    std::vector<double> ComputeKinematics::vectorAddition(std::vector<double> a, std::vector<double> b)
    {
        std::vector<double> sum(a.size());
        for(int i=0; i<a.size(); i++){
            sum[i] = a[i] + b[i];
        }
        return sum;
    }

    std::vector<double> ComputeKinematics::vectorSubtraction(std::vector<double> a, std::vector<double> b)
    {
        std::vector<double> difference(a.size());
        for(int i=0; i<a.size(); i++){
            difference[i] = a[i] - b[i];
        }
        return difference;
    }

std::vector<double> ComputeKinematics::getLinkParamFromQ2(const double& q_2)
{
const double a = 140;
const double b = 50;
const double c = 30;
const double d = 60;
const double lo = 166;
const double w = 160;

const double a_1 = a + lo - b;
const double a_2 = lo + d;
const double a_3 = w;
const double a_4 = c;

const double b_1 = pow(a_1,2) + pow(a_2,2) + pow(a_3,2) + pow(a_4,2);
const double b_2 = 2*(a_1*a_2 + a_3*a_4);
const double b_3 = 2*(-a_2*a_4 + a_1*a_3);

const double x_0 = sqrt(pow(w-c,2) + pow(a-b-d,2));

const double chest_depth = 110; //Σ4からΣ5までの鉛直方向の距離
const double chest_wide = 336; //Σ4からΣ5までの水平方向の距離
const double chest_alpha = atan2(336, 110); //Σ4からΣ5までの角度

double theta_3 = M_PI/2 - q_2; 
double x = 0; //台形ねじの長さ,つまりlink_param_[3]
double phi = 0; //鉛直方向からのlink_param_[4]の角度
double theta_4 = 0; //link_param_theta_[4]


x = a_1*sin(q_2) - a_4*cos(q_2) + sqrt(pow(a_1*sin(q_2) - a_4*cos(q_2),2) - (pow(a_1,2) + pow(a_4,2) - pow(a_2,2) - pow(a_3,2)));


phi = acos((b_1 - pow(x,2))/sqrt(pow(b_2,2) + pow(b_3,2))) - ComputeKinematics::atan2ChangeDomain(b_3,b_2);
theta_4 = -(chest_alpha + phi + M_PI/2 - q_2);

std::vector<double> link_param_2_to_4 = {theta_3, x, theta_4};

chest_angle_ = phi;
return link_param_2_to_4;
}


Eigen::MatrixXd ComputeKinematics::computeT(const Eigen::VectorXd& q)
{
/*q[2]からlink_param_[2],link_param_[3],link_param_[4]への変換式*/
std::vector<double> link_param_2_to_4 = ComputeKinematics::getLinkParamFromQ2(q(2));

std::vector<double> link_param_a = {443, 40, 150, link_param_2_to_4.at(1), 353.548, 0, 0}; //DH法のパラメータa[i],
std::vector<double> link_param_alpha = {0, -M_PI/2, -M_PI/2, 0, 0, M_PI/2, M_PI/2}; //DH法のパラメータα[i]
std::vector<double> link_param_d = {-425, -36.8, 850, 0, 0, 0, 0}; //DH法のパラメータd[i], 5/18追記: 3つめのパラメータ未定
std::vector<double> link_param_theta = {q(0), q(1)-M_PI/2, link_param_2_to_4.at(0), link_param_2_to_4.at(2), atan2(336, 110), M_PI/2, 0}; //DH法のパラメータθ[i]
Eigen::MatrixXd Tdh[7] = {Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4)}; //同次変換行列

for(int i=0; i<7; i++){
Tdh[i] << cos(link_param_theta[i]), -sin(link_param_theta[i]), 0, link_param_a[i],
cos(link_param_alpha[i])*sin(link_param_theta[i]), cos(link_param_alpha[i])*cos(link_param_theta[i]), -sin(link_param_alpha[i]), -link_param_d[i]*sin(link_param_alpha[i]),
sin(link_param_alpha[i])*sin(link_param_theta[i]), sin(link_param_alpha[i])*cos(link_param_theta[i]), cos(link_param_alpha[i]), link_param_d[i]*cos(link_param_alpha[i]),
0, 0, 0, 1;
}


Eigen::MatrixXd T = Tdh[0] * Tdh[1] * Tdh[2] * Tdh[3] * Tdh[4] * Tdh[5] * Tdh[6]; //基準座標系から手先までの同次変換行列

return T;
}

Eigen::MatrixXd ComputeKinematics::computeJ(const Eigen::VectorXd& q)
{
double diff = 0.001; //ヤコビ行列導出のための偏微分に使う微小変位
Eigen::VectorXd d_q[3] = {Eigen::VectorXd::Constant(3,0), Eigen::VectorXd::Constant(3,0), Eigen::VectorXd::Constant(3,0)};
d_q[0] << diff, 0, 0; //関節q1の微小変位
d_q[1] << 0, diff, 0; //関節q2の微小変位
d_q[2] << 0, 0, diff; //関節q3の微小変位

Eigen::MatrixXd T[4] = {Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4), Eigen::MatrixXd::Zero(4, 4)};
T[0] = ComputeKinematics::computeT(q);
for(int i=0; i<3; i++)
{
    T[i+1] = ComputeKinematics::computeT(q.array() + d_q[i].array());
}

Eigen::VectorXd r[4] = {Eigen::VectorXd::Constant(3,0), Eigen::VectorXd::Constant(3,0), Eigen::VectorXd::Constant(3,0), Eigen::VectorXd::Constant(3,0)};
for(int i=0; i<4; i++)
{
    double phi = atan2(T[i](1,0), T[i](0,0));
    double theta = ComputeKinematics::atan2ChangeDomain(-T[i](2,0), sqrt(pow(T[i](2,1), 2) + pow(T[i](2,2), 2)));
    double psi = ComputeKinematics::atan2ChangeDomain(T[i](2,1), T[i](2,2));
    if(psi > M_PI)
    {
        psi -= 2*M_PI;
    }
    r[i] << phi, theta, psi;
    std::cout << "r[" << i << "]: " << r[i] << std::endl;
}

Eigen::MatrixXd J_r = Eigen::MatrixXd::Zero(3, 3); //ヤコビ行列
for(int i=0; i<3; i++)
{
    for(int j=0; j<3; j++)
    {
        J_r(j,i) = (r[i+1](j)-r[0](j)) / diff;
        // if(fabs(J_r(j,i))<0.000001)
        // {
        //     J_r(j,i)=0;
        // }
    }
}
std::cout << J_r << std::endl;

return J_r;
}

Eigen::VectorXd ComputeKinematics::getPosture(const Eigen::VectorXd& q)
{
    Eigen::MatrixXd T = ComputeKinematics::computeT(q);
    double phi = atan2(T(1,0), T(0,0));
    double theta = ComputeKinematics::atan2ChangeDomain(-T(2,0), sqrt(pow(T(2,1), 2) + pow(T(2,2), 2)));
    double psi = ComputeKinematics::atan2ChangeDomain(T(2,1), T(2,2));
    if(psi > M_PI)
    {
        psi -= 2*M_PI;
    }

    Eigen::VectorXd r = Eigen::VectorXd::Constant(3,0);
    r << phi, theta, psi;
    return r;
}

Eigen::VectorXd ComputeKinematics::computePosture(const Eigen::VectorXd& r_goal) //目標の手先位置姿勢ベクトルから関節変位ベクトルを算出する関数    
{
    Eigen::VectorXd current_position = Eigen::VectorXd::Constant(3,0);
    current_position << 0, 0.523597, 0.226798;

    //Eigen::VectorXd q_0 = Eigen::VectorXd::Constant(3,0);
    //q_0 << 0*M_PI/180, 30*M_PI/180, 12.96*M_PI/180; //初期関節変位ベクトル
    
    Eigen::VectorXd r_0 = ComputeKinematics::getPosture(current_position);
    if (abs(r_goal(2)) > M_PI)
    {
        r_0(2) += 2*M_PI;
    }    
    int divisor = 5;
    Eigen::VectorXd d_r = (r_goal.array() - r_0.array()) / divisor; //オイラー法で逆運動学を解く際の微小手先位置姿勢ベクトル
    std::cout << "d_r: " << d_r.array() << std::endl;
    Eigen::VectorXd q = current_position; //数値計算過程における関節角度
    Eigen::VectorXd r = r_0; //数値計算過程における位置姿勢
    Eigen::MatrixXd J_r = Eigen::MatrixXd::Zero(3,3); //数値計算過程におけるヤコビ行列
    Eigen::VectorXd d_q = Eigen::VectorXd::Constant(3,0); //オイラー法で逆運動学を解く際の微小関節変位ベクトル
    for(int i=0; i<divisor; i++)
    {
        r = ComputeKinematics::getPosture(q);
        J_r = ComputeKinematics::computeJ(q);
        d_q = J_r.inverse() * d_r;
        std::cout << "d_q: " << d_q << std::endl;
        q = q + d_q;
        /*応急処置的な処理*/
        /*q_[i].resize(3);
        r_[i].resize(3);
        for(int j=0; j<3; j++)
        {
            q_[i].at(j) = q(j);
        }
        r_[i].at(0) = r(0)*180/M_PI;
        r_[i].at(1) = r(1)*180/M_PI - 270;
        r_[i].at(2) = r(2)*180/M_PI - 180;*/

        r = getPosture(q);
        //std::cout <<" q0: " << q_[i].at(0) << " q1: " << q_[i].at(1) << " q2: " << q_[i].at(2) << std::endl;
        std::cout <<" phi: " << r(0) << " theta: " << r(1) << " psi: " << r(2) << std::endl;
        std::cout <<" q(0): " << q(0) << " q(1): " << q(1) << " q(2): " << q(2) << std::endl;
    }
    std::cout << "q: " <<  q << std::endl;
    r_ = getPosture(q);
    return q;
}

Eigen::VectorXd ComputeKinematics::ConvertMotionAxisIntoPosture(const double& roll, const double& pitch)
{
    Eigen::Vector3d x(1,0,0); //手先座標系のx軸を基準座標系からみたときのx座標, y座標, z座標
    Eigen::Vector3d y(0,1,0); //手先座標系のy軸を基準座標系からみたときのx座標, y座標, z座標
    Eigen::Vector3d z(0,0,0); //手先座標系のz軸を基準座標系からみたときのx座標, y座標, z座標
    y(0) = - sin(roll); //座標の高さ成分
    x(0) = cos(pitch);
    if(roll < 10*M_PI/180) //座標の奥行成分の決定(手前が正)
    {
        y(2) = 0;
    }
    else if(roll < 20*M_PI/180)
    {
        y(2) = -sin(10*M_PI/180);
    }
    else if(roll < 30*M_PI/180)
    {
        y(2) = -sin(15*M_PI/180);
    }
    else
    {
        std::cout << "Error: Inputed number is out of range" << std::endl;
    }

    y(1) = sqrt(1 - (pow(y(0),2) + pow(y(2),2))); //座標の水平成分, これでy座標の全成分がさだまった

    double c_1[2] = {y(2)/y(1), x(0)*y(0)/y(1)};
    double c_2[2] = {c_1[0]*c_1[1]/(1 + pow(c_1[0],2)), (pow(x(0),2) + pow(c_1[1],2) - 1)/(1 + pow(c_1[0],2))};
    x(2) = -c_2[0] - sqrt(pow(c_2[0],2) - c_2[1]);
    x(1) = -c_1[0] * x(2) - c_1[1];
     
    z = x.cross(y);
    std::cout << "x" << std::endl << x << std::endl << "y" << std::endl << y << std::endl << "z" << std::endl << z << std::endl; 

    double phi = atan2(x(1), x(0));
    double theta = atan2ChangeDomain(-x(2), sqrt(pow(y(2), 2) + pow(z(2), 2)));
    double psi = atan2ChangeDomain(y(2), z(2));

    Eigen::VectorXd r = Eigen::VectorXd::Constant(3,0);
    r << phi, theta, psi;
    std::cout << "r_goal" << std::endl << r << std::endl;
    return r;
}

Eigen::VectorXd ComputeKinematics::inputGoal()
{
Eigen::VectorXd r_goal = Eigen::VectorXd::Constant(3, 0);
std::cout << "Input r_goal" << std::endl;
std::cout << "phi: ";
std::cin >> r_goal(0);
std::cout << "theta: ";
std::cin >> r_goal(1);

r_goal(1) = r_goal(1) + 270;
if(r_goal(0) < 10)
{
    r_goal(2) = 180;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else if(r_goal(0) < 20){
    r_goal(2) = 175;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else if(r_goal(0) < 30){
    r_goal(2) = 170;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else{
    r_goal(2) = 165;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}

for(int i=0; i<3; i++) //度数法から弧度法に変換
{
    r_goal(i) = r_goal(i) * M_PI/180;
}
return r_goal;
}

Eigen::VectorXd ComputeKinematics::inputGoalFromGui()
{
Eigen::VectorXd r_goal = Eigen::VectorXd::Constant(3, 0);
r_goal(0) = gui_goal_(0);
r_goal(1) = gui_goal_(1) + 270;

if(r_goal(0) < 10)
{
    r_goal(2) = 180;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else if(r_goal(0) < 20){
    r_goal(2) = 175;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else if(r_goal(0) < 30){
    r_goal(2) = 170;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}else{
    r_goal(2) = 165;
    std::cout << "r_goal(0): " << r_goal(0) << "r_goal(1): " << r_goal(1) << "r_goal(2): " << r_goal(2) << std::endl;
}

for(int i=0; i<3; i++) //度数法から弧度法に変換
{
    r_goal(i) = r_goal(i) * M_PI/180;
}
return r_goal;
}
    


};
