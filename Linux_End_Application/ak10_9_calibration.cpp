#include "ak10_9_calibration.hpp"

AK10_9 hAKMotorLeftHip;

AK10_9::AK10_9(void):position(0.0), velocity(0.0), current(0.0), temperature(0.0), systemTime(0), index(0)
{}

void AK10_9::GetFeedbackCargo(uint8_t * data)
{
    memcpy(&position, data + 8, 4);
    memcpy(&velocity, data + 12, 4);
    memcpy(&current, data + 16, 4);
    memcpy(&index, data, 4);
    memcpy(&systemTime, data + 4, 4);

    std::cout << "Index:" << index << "      " << "Time:" << systemTime << "      " <<  "Position:" << position << "      " << "Velocity: " << velocity << "      " << "Current: " << current << std::endl;
}
