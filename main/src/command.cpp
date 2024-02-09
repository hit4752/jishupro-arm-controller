#include "command.hpp"
#include "time.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

struct velControlArgs {
    Motor* motor;
    double vel;
};

struct posControlArgs {
    Motor* motor;
    double* pos;
};

struct faceControlArgs {
    Motor* motor;
    double* pos;
};

void Command::init()
{
    // this->motor_num = this->kinematics->getMotorNum();

    this->cmd_map["a"] = [this]() {
        if (argc < 1) {
            argv[1] = "a";
            argc++;
        }

        uint8_t target = 0x00;
        if (argv[1] == "a") {
            target = 0x0f;
        } else {
            target = (1 << std::stoi(argv[1]));
        }

        for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
            if (target & (1 << i)) {
                this->kinematics->getMotor(i)->setTargetCurrent(0);
                this->kinematics->getMotor(i)->setActivateFlag();
            }
        }
    };

    this->cmd_map["f"] = [this]() {
        if (argc < 1) {
            argv[1] = "a";
            argc++;
        }

        uint8_t target = 0x00;
        if (argv[1] == "a") {
            target = 0x0f;
        } else {
            target = (1 << std::stoi(argv[1]));
        }

        for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
            if (target & (1 << i)) {
                this->kinematics->getMotor(i)->setTargetCurrent(0);
                this->kinematics->getMotor(i)->setStopFlag();
            }
        }
    };

    this->cmd_map["c"] = [this]() {
        if (argc < 1) {
            argv[1] = "a";
            argc++;
        }

        if (argc < 2) {
            argv[2] = "0";
            argc++;
        }

        uint8_t target = 0x00;
        if (argv[1] == "a") {
            target = 0x0f;
        } else {
            target = (1 << std::stoi(argv[1]));
        }

        int16_t current = std::stoi(argv[2]);

        for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
            if (target & (1 << i)) {
            }
            this->kinematics->getMotor(i)->setTargetCurrent(current);
        }
    };

    this->cmd_map["set"] = [this]() {
        if (argc < 1) {
            argv[1] = "a";
            argc++;
        }

        uint8_t target = 0x00;
        if (argv[1] == "a") {
            target = 0x0f;
        } else {
            target = (1 << std::stoi(argv[1]));
        }

        for (int i = 0; i < this->kinematics->getMotorNum(); i++) {
            if (target & (1 << i)) {
                this->kinematics->getMotor(i)->setEncPosOffset(this->kinematics->getMotor(i)->getEncPos());
                printf("motor %d: offset = %ld\n", i, this->kinematics->getMotor(i)->getEncPosOffset());
            }
        }
    };

    this->cmd_map["vel"] = [this]() {
        if (argc < 1) {
            printf("usage: vel [motor] [velocity]\n");
            return;
        }

        if (argc < 2) {
            argv[2] = "0";
            argc++;
        }

        uint8_t id = std::stoi(argv[1]);

        if (this->kinematics->getMotor(id) == nullptr) {
            printf("motor %d does not exist.\n", id);
            return;
        }

        double vel = std::stod(argv[2]);

        pthread_t vel_control_thread;
        velControlArgs args = {this->kinematics->getMotor(id), vel};
        pthread_create(
            &vel_control_thread,
            NULL,
            [](void* args) -> void* {
                Motor* motor = ((velControlArgs*)args)->motor;
                double vel = ((velControlArgs*)args)->vel;

                double k_p = 2000.0;
                double k_i = 1.0;
                double k_d = 0.0;

                double t_prev = timer.getTime();
                double e_prev = 0.0;
                double e_int = 0.0;

                while (1) {
                    double t = timer.getTime();
                    double dt = t - t_prev;
                    t_prev = t;

                    while (!motor->getUpdateFlag()) {
                        if (timer.getTime() - t > 100.0) {
                            printf("motor update timeout\n");
                            motor->setTargetCurrent(0);
                            while (1)
                                ;
                        }
                    }
                    motor->resetUpdateFlag();

                    if (timer.getTime() - t < 1.0) {
                        continue;
                    }

                    double e = vel - motor->getVel();
                    e_int += e * dt / 1000.0;
                    e_int *= 0.99;
                    double e_diff = (e - e_prev) / (dt / 1000.0);
                    e_prev = e;
                    double current = e * k_p + e_int * k_i + e_diff * k_d;
                    current = std::clamp(current, -3000.0, 3000.0);
                    motor->setTargetCurrent(current);
                    printf("pos = %9.5f, vel = %9.5f, time = %7.3f, current = %9.2f\n", motor->getPos(), motor->getVel(), dt, current);
                }

                return NULL;
            },
            (void*)&args);

        std::string s;
        std::cin >> s;
        pthread_cancel(vel_control_thread);
        timer.delay(100);
        this->kinematics->getMotor(id)->setTargetCurrent(0);
    };

    this->cmd_map["pos"] = [this]() {
        if (argc < 1) {
            printf("usage: pos [motor] [position]\n");
            return;
        }

        if (argc < 2) {
            argv[2] = "0";
            argc++;
        }

        uint8_t id = std::stoi(argv[1]);

        if (this->kinematics->getMotor(id) == nullptr) {
            printf("motor %d does not exist.\n", id);
            return;
        }

        double pos = this->kinematics->getMotor(id)->getPos() + std::stod(argv[2]);

        pthread_t pos_control_thread;
        posControlArgs args = {this->kinematics->getMotor(id), &pos};
        pthread_create(
            &pos_control_thread,
            NULL,
            posControl,
            (void*)&args);

        std::string s;
        std::cin >> s;
        pthread_cancel(pos_control_thread);
        timer.delay(100);
        this->kinematics->getMotor(id)->setTargetCurrent(0);
    };

    this->cmd_map["auto"] = [this]() {

        uint8_t id = 0;

        if (this->kinematics->getMotor(id) == nullptr) {
            printf("motor %d does not exist.\n", id);
            return;
        }

        double pos = 0.0;

        pthread_t face_position_thread;
        faceControlArgs fargs = {this->kinematics->getMotor(id), &pos};
        pthread_create(
            &face_position_thread,
            NULL,
            [](void* args) -> void* {
                Motor* motor = ((faceControlArgs*)args)->motor;
                double* pos = ((faceControlArgs*)args)->pos;

                double t;
                double t_recv;
                double t_offset;

                double pos_prev = motor->getPos();
                double pos_new = pos_prev;

                while (1) {
                    t = timer.getTime();

                    int fd = open("/mnt/c/Users/PC/Documents/head_pose_detection/head_pose.txt", O_RDONLY);

                    if (fd == -1) {
                        printf("[face] file not found\n");
                    } else {
                        int yaw, pitch, roll;
                        double time;
                        char buf[100];
                        int len = read(fd, buf, sizeof(buf));
                        sscanf(buf, "%d %d %d %lf", &yaw, &pitch, &roll, &time);

                        if (std::abs(time - t_recv) > 1.0) {
                            t_recv = time;
                            t_offset = t - time;
                            pos_new = std::clamp(pos_prev + DEG2RAD(-yaw), DEG2RAD(-70.0), DEG2RAD(70.0));
                            printf("[face] yaw: %d, pos: %9.5f, dt = %9.5f\n", yaw, pos_new, time - t_recv);
                        } else {
                            printf("[face] timeout\n");
                        }

                        *pos = 0.5 * pos_new + 0.5 * (*pos);
                    }

                    pos_prev = motor->getPos();

                    while (timer.getTime() - t < 400.0) {
                    }
                }
            },
            (void*)(&fargs));

        pthread_t pos_control_thread;
        posControlArgs args = {this->kinematics->getMotor(id), &pos};
        pthread_create(
            &pos_control_thread,
            NULL,
            posControl,
            (void*)(&args));

        std::string s;
        std::cin >> s;
        pthread_cancel(pos_control_thread);
        pthread_cancel(face_position_thread);
        timer.delay(100);
        this->kinematics->getMotor(id)->setTargetCurrent(0);
    };
}

void Command::execute(std::string cmd)
{
    this->command = cmd;

    this->argc = 0;
    this->argv[0] = "";

    for (int i = 0; i < this->command.length(); i++) {
        if (this->command[i] == ' ') {
            if (this->argv[this->argc] != "") {
                this->argc++;
                this->argv[this->argc] = "";
            }
        } else {
            this->argv[this->argc] += this->command[i];
        }
    }

    while (this->argv[this->argc] == "") {
        this->argc--;
    }

    if (this->cmd_map.find(this->argv[0]) != this->cmd_map.end()) {
        this->cmd_map[this->argv[0]]();
    } else {
        printf("%s: command not found\n", this->argv[0].c_str());
    }
}

void* posControl(void* args) {
    Motor* motor = ((posControlArgs*)args)->motor;
    double* pos = ((posControlArgs*)args)->pos;

    double k_p = 7000.0;
    double k_i = 300.0;
    double k_d = 600.0;

    double t_prev = timer.getTime();
    double e_prev = 0.0;
    double e_int = 0.0;

    bool dir = true;
    if (*pos < motor->getPos()) {
        dir = false;
    }

    while (1) {

        while (!motor->getUpdateFlag()) {
            if (timer.getTime() - t_prev > 1000.0) {
                printf("motor update timeout\n");
                motor->setTargetCurrent(0);
                while (1)
                    ;
            }
        }
        motor->resetUpdateFlag();

        double t = timer.getTime();
        double dt = t - t_prev;
        t_prev = t;

        if (dt < 1.0) {
            continue;
        }

        if (*pos - motor->getPos() > 0.0) {
            dir = true;
        } else if (*pos - motor->getPos() < -0.0) {
            dir = false;
        }

        double e = *pos - motor->getPos();
        e_int += e * dt / 1000.0;
        e_int *= 0.999;
        double e_diff = (e - e_prev) / (dt / 1000.0);
        e_prev = e;
        double current = e * k_p + e_int * k_i + e_diff * k_d;
        current = std::clamp(current, -2000.0, 2000.0);
        current += -750 * std::sin(motor->getPos());
        if (dir) {
            current += 500;
        } else {
            current -= 500;
        }
        current = std::clamp(current, -10000.0, 10000.0);
        motor->setTargetCurrent(current);
        printf("[pos] pos = %9.5f, %9.5f, vel = %9.5f, time = %7.3f, current = %9.2f, %9.2f, %9.2f\n", motor->getPos(), e, motor->getVel(), dt, current, e_int * k_i, -500 * std::sin(motor->getPos()));
    }

    return NULL;
}