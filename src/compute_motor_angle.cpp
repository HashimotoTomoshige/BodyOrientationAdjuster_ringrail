#include "compute_motor_angle.h"

namespace robot_posture_control
{
    ComputeMotorAngle::ComputeMotorAngle(){}

    /*腰部回旋機構の関節角をモーター角度に変換(まっさんのスプレッドシート"計算表/腰部廻旋機構"参照)*/
    int ComputeMotorAngle::convertWaistAngleToRotationNumber(const double& theta)
    {
        double d_x = 0; //台形ねじの変位
        const int a = 130;      //部材の長さ
        const int d = 100;

        d_x = a * sin(theta) + d - sqrt(pow(d,2) - (a * (1 - cos(theta)) * a * (1 - cos(theta))));
        if(d_x < 0)
        {
            std::cout << "----------Error: dx is out of range-----------" << std::endl;
            d_x = 0;
        }
        int motor_position = -d_x / 3 * 500;
        std::cout << "ComputeMotorAngle::convertWaistAngleToRotationNumber()" << std::endl;
        std::cout << "motor_position: " << motor_position << std::endl << "d_x: " << d_x << " theta: " << theta << std::endl;
        return motor_position;
    }

    /*屈曲機構の関節角をモーター角度に変換(まっさんのスプレッドシート"計算表/屈曲機構"参照)*/
    int ComputeMotorAngle::convertBendingAngleToRotationNumber(const double& theta)
    {
        int rotation_number = 0; //モーターの回転数
        double d_x = 0; //台形ねじの変位
        double x = 0; //台形ねじの長さ
        const int a = 340;
        const int b = 120;
        const int la = 170;
        double theta_2 = atan2(la * sin(theta + M_PI/4) - b, la * cos(theta + M_PI/4) + a);
        x = a * cos(theta_2) - b*sin(theta_2) + sqrt(pow(a * cos(theta_2) - b * sin(theta_2), 2) + pow(la, 2) - pow(a, 2) - pow(b, 2));
        const double x_0 = a + sqrt(pow(la, 2) - pow(b, 2));
        d_x = -x + x_0;
        if(d_x < 0)
        {
            std::cout << "----------Error: dx is out of range-----------" << std::endl;
            d_x = 0;
        }
        std::cout << "ComputeMotorAngle::convertBendingAngleToRotationNumber()" << std::endl;
        int motor_position = d_x / 3 * 500;
        std::cout << "motor_position: " << motor_position << std::endl << "d_x: " << d_x << " theta: " << theta << std::endl;
        return motor_position;
    }

    /*胸部回旋機構の関節角をモーター角度に変換(まっさんのスプレッドシート"計算表/胸部回旋機構"参照)*/
    int ComputeMotorAngle::convertChestAngleToRotationNumber(const double& theta)
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

        double x = 0; //台形ねじの長さ
        double d_x = 0; //台形ねじの変位

        x = a_1*sin(theta) - a_4*cos(theta) + sqrt(pow(a_1*sin(theta) - a_4*cos(theta),2) - (pow(a_1,2) + pow(a_4,2) - pow(a_2,2) - pow(a_3,2)));
        d_x = x - x_0;
        if(d_x < 0)
        {
            std::cout << "----------Error: dx is out of range-----------" << std::endl;
            d_x = 0;
        }
        int motor_position = -d_x / 3 * 500;

        std::cout << "ComputeMotorAngle::convertChestAngleToRotationNumber()" << std::endl;
        std::cout << "motor_position: " << motor_position << std::endl << "d_x: " << d_x << " theta: " << theta << std::endl;
        return motor_position;
    }

    int ComputeMotorAngle::convertBaseAngleToRotationNumber(const double& theta)
    {
        const double x_0 = 220; //台形ねじの初期値
        const double a = 230;
        const double b = 250;
        const double d = 20;
        const double r = sqrt(pow(a,2) + pow(b,2)) - d;

        double x = 0; //台形ねじの長さ
        double d_x = 0; //台形ねじの変位

        x = sqrt(pow(x_0 - r*sin(theta),2) + pow(r - r*cos(theta),2));
        d_x = x - x_0;
        int motor_position = d_x / 3 * 500;
        //std::cout << "ComputeMotorAngle::convertBaseAngleToRotationNumber()" << std::endl;
        //std::cout << "d_x: " << d_x << std::endl;
        return motor_position;
    }

};