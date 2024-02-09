#pragma once

#include <termios.h>
#include <cstdint>
#include "kinematics.hpp"

enum class Commands : uint8_t {
    STOP,
    CONNECT,
    CALIB,
    CURRENT,
    ACTIVATE = 0x0f,
    FEEDBACK = 0x10,
};

#define ID(x) (1 << x)

class Serial
{

public:
    Serial(Kinematics* kinematics) : kinematics(kinematics) {}
    ~Serial() {}

    void findPort(int);
    void transmit(Commands, uint8_t, uint8_t[8]);
    void receive();
    void transmitCurrent();
    void closePort();

private:
    int fd;
    struct termios tio;

    uint8_t tx_buf[11];
    uint8_t rx_buf[11];

    Commands command;
    uint8_t target;
    uint8_t data[8];

    bool connected = false;
    
    Kinematics* kinematics;
    // int motor_num;
};