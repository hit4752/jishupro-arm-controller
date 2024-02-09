#include "main.hpp"
#include "serial.hpp"
#include "motor.hpp"
#include "time.hpp"
#include "kinematics.hpp"
#include "command.hpp"
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <functional>
#include <iostream>
#include <string>


struct serialLoopArgs {
    Serial* serial;
    Timer* timer;
};

// void* serialLoop(void* args)
// {
//     Serial* serial = ((serialLoopArgs*)args)->serial;
//     Timer* timer = ((serialLoopArgs*)args)->timer;

//     while (1) {
//         double s = timer->getTime();

//         serial->receive();
//         serial->transmitCurrent();

//         printf("elapsed time: %f ms\n", timer->getTime() - s);

//         while (timer->getTime() - s < 2.0f) {
//         }
//     }
// }

int main(int argc, char** argv)
{
    timer.start();

    Motor motor0(0, 16384, 16.0f);
    Motor motor1(1, 16384, 64.0f);
    Motor motor2(2, 16384, 16.0f);

    Kinematics kinematics(3, &motor0, &motor1, &motor2, nullptr);

    Serial serial(&kinematics);
    Command command(&kinematics);
    command.init();
    
    serial.findPort(B38400);
    serial.transmit(Commands::ACTIVATE, ID(0) | ID(1) | ID(2), NULL);

    // std::function<void*(void*)> serialLoop = [](void* args) -> void* {
    //     Serial* serial = ((serialLoopArgs*)args)->serial;
    //     Timer* timer = ((serialLoopArgs*)args)->timer;

    //     while (1) {
    //         double s = timer->getTime();

    //         serial->receive();
    //         serial->transmitCurrent();

    //         // printf("elapsed time: %f ms\n", timer->getTime() - s);

    //         while (timer->getTime() - s < 2.0f) {
    //         }
    //     }

    //     return NULL;
    // };

    serialLoopArgs args = { &serial, &timer };
    pthread_t serial_thread;
    pthread_create(
        &serial_thread,
        NULL,
        [](void* args) -> void* {
            Serial* serial = ((serialLoopArgs*)args)->serial;
            Timer* timer = ((serialLoopArgs*)args)->timer;

            while (1) {
                double s = timer->getTime();

                serial->receive();
                serial->transmitCurrent();

                // printf("elapsed time: %f ms\n", timer->getTime() - s);

                while (timer->getTime() - s < 2.0f) {
                }
            }

            return NULL;
        },
        (void*)&args);

    while (1) {
        std::cout << "> ";
        std::string str;
        std::getline(std::cin, str);
        // std::cout << str << std::endl;
        command.execute(str);
    }

    serial.closePort();
    return 0;
}