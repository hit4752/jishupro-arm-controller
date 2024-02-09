#pragma once

#include "motor.hpp"

class Kinematics
{
public:
    Kinematics(int num, Motor* motor0, Motor* motor1, Motor* motor2, Motor* motor3)
    {
        this->motor_num = num;
        this->motors[0] = motor0;
        this->motors[1] = motor1;
        this->motors[2] = motor2;
        this->motors[3] = motor3;
    }

    ~Kinematics() {}

    Motor* getMotor(uint8_t id)
    {
        if (id < this->motor_num) {
            return this->motors[id];
        }

        return nullptr;
    }

    int getMotorNum() const
    {
        return this->motor_num;
    }

private:
    Motor* motors[4];
    int motor_num;
};