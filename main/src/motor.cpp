#include "motor.hpp"
#include "time.hpp"
#include <cmath>

void Motor::update(uint8_t data[8])
{
    this->enc_pos = static_cast<int32_t>((data[0] << 24) | (data[1] << 16) | (data[2] << 8)) / 256;
    this->enc_vel = static_cast<int16_t>((data[3] << 8) | data[4]);
    this->actual_current = static_cast<int16_t>((data[5] << 8) | data[6]);

    this->pos = (this->enc_pos - this->enc_pos_offset) * 2.0f * M_PI / this->enc_resolution / this->gear_ratio;
    this->vel = this->enc_vel * 8000 * 2.0f * M_PI / this->enc_resolution / this->gear_ratio;

    this->update_flag = true;
}