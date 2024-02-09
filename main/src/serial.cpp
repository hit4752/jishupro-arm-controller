#include "serial.hpp"
#include "motor.hpp"
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>

void Serial::findPort(int baud_rate)
{
    // while (1) {
    //     int fd = open("/mnt/c/Users/PC/Documents/head_pose_detection/head_pose.txt", O_RDONLY);

    //     if (fd == -1) {
    //         printf("file not found\n");
    //         usleep(1000000);
    //     }

    //     int yaw, pitch, roll;
    //     char buf[100];
    //     int len = read(fd, buf, sizeof(buf));
    //     sscanf(buf, "%d %d %d", &yaw, &pitch, &roll);
    //     printf("len = %d, yaw: %d, pitch: %d, roll: %d\n", len, yaw, pitch, roll);
    // }

    this->tio.c_cflag += CREAD;
    this->tio.c_cflag += CLOCAL;
    this->tio.c_cflag += CS8;
    this->tio.c_cflag += 0;
    this->tio.c_cflag += 0;

    cfsetispeed(&this->tio, baud_rate);
    cfsetospeed(&this->tio, baud_rate);

    cfmakeraw(&this->tio);

    while (1) {
        printf("connecting...\n");

        for (int i = 0; i < 255; i++) {

            char port[20];
            snprintf(port, sizeof(port), "/dev/ttyS%d", i);
            this->fd = open(port, O_RDWR);
            if (this->fd < 0) {
                continue;
            }

            printf("port open: %s\n", port);

            tcsetattr(this->fd, TCSANOW, &this->tio);
            ioctl(this->fd, TCSETS, &this->tio);

            int val = 1;
            ioctl(this->fd, FIONBIO, &val);

            this->transmit(Commands::CONNECT, 0, NULL);

            usleep(100000);

            this->receive();
            if (this->connected) {
                printf("device found: %s\n", port);
                return;
            } else {
                close(this->fd);
            }
        }

        printf("device not found\n");
        usleep(1000000);
    }
    while(1);
}

void Serial::transmit(Commands command, uint8_t target, uint8_t data[8])
{
    tx_buf[0] = (uint8_t)command;
    tx_buf[1] = target;
    tx_buf[10] = '\n';

    if (data != NULL) {
        memcpy(&tx_buf[2], data, 8);
    } else {
        memset(&tx_buf[2], 0, 8);
    }

    write(this->fd, this->tx_buf, sizeof(this->tx_buf));
}

void Serial::receive()
{
    while (1) {
        int len = read(this->fd, this->rx_buf, sizeof(this->rx_buf));
        // printf("len: %d\n", len);
        if ((len < (int)(sizeof(this->rx_buf))) || (*(this->rx_buf + sizeof(this->rx_buf) - 1) != '\n')) {
            return;
        }

        this->command = (Commands)this->rx_buf[0];
        this->target = this->rx_buf[1];
        memcpy(this->data, &this->rx_buf[2], 8);

        switch (this->command) {

            case Commands::CONNECT:
                this->connected = true;
                break;

            case Commands::FEEDBACK:
                for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
                    if (this->kinematics->getMotor(i) != nullptr && (this->target & ID(i))) {
                        this->kinematics->getMotor(i)->update(this->data);
                    }
                }
                break;
        }
    }
}

void Serial::transmitCurrent()
{
    for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
        if (this->kinematics->getMotor(i)->getActivateFlag()) {
            this->transmit(Commands::ACTIVATE, ID(i), NULL);
            this->kinematics->getMotor(i)->resetActivateFlag();
        }

        if (this->kinematics->getMotor(i)->getStopFlag()) {
            this->transmit(Commands::STOP, ID(i), NULL);
            this->kinematics->getMotor(i)->resetStopFlag();
        }
    }
    uint8_t target = 0x00;
    uint8_t data[8] = {0};
    for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
        target |= ID(i);
        int16_t current = this->kinematics->getMotor(i)->getTargetCurrent();
        data[2 * i] = current >> 8;
        data[2 * i + 1] = current & 0xFF;
    }
    this->transmit(Commands::CURRENT, target, data);
}

void Serial::closePort()
{
    close(this->fd);
}