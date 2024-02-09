#pragma once

#include <cstdint>
#include <string>
#include <map>
#include <functional>
#include "kinematics.hpp"

class Command
{
public:
    Command(Kinematics* kinematics) : kinematics(kinematics) {}
    ~Command() {}

    void init();
    void execute(std::string);

private:
    std::string command;
    std::string argv[8];
    int argc;

    std::map<std::string, std::function<void(void)>> cmd_map;

    Kinematics* kinematics;
    // int motor_num;
};

void* posControl(void*);