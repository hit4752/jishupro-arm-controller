#pragma once

#include <cstdint>

class Motor
{
private:
    uint8_t id;
    int64_t enc_pos;
    int64_t enc_pos_offset;
    int64_t enc_vel;
    int16_t actual_current;

    int16_t target_current;

    int64_t enc_resolution = 16384;
    double gear_ratio = 1.0f;

    double pos;  // [rad]
    double vel;  // [rad/s]

    bool activate_flag = false;
    bool stop_flag = false;

    bool update_flag = false;

public:
    Motor(uint8_t id, int64_t enc_resolution, double gear_ratio)
        : id(id), enc_resolution(enc_resolution), gear_ratio(gear_ratio) {}

    void update(uint8_t[8]);

    void setTargetCurrent(int16_t current) { this->target_current = current; }
    void setEncPosOffset(int64_t offset) { this->enc_pos_offset = offset; }
    void setActivateFlag() { this->activate_flag = true; }
    void resetActivateFlag() { this->activate_flag = false; }
    void setStopFlag() { this->stop_flag = true; }
    void resetStopFlag() { this->stop_flag = false; }
    void setUpdateFlag() { this->update_flag = true; }
    void resetUpdateFlag() { this->update_flag = false; }

    int16_t getTargetCurrent() const { return this->target_current; }
    int64_t getEncPos() const { return this->enc_pos; }
    int64_t getEncPosOffset() const { return this->enc_pos_offset; }
    int64_t getEncVel() const { return this->enc_vel; }
    double getPos() const { return this->pos; }
    double getVel() const { return this->vel; }
    bool getActivateFlag() const { return this->activate_flag; }
    bool getStopFlag() const { return this->stop_flag; }
    bool getUpdateFlag() const { return this->update_flag; }
};