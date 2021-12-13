#ifndef AK10_9_CALIBRATION
#define AK10_9_CALIBRATION

#include "mfec_communication.hpp"

class AK10_9
{
    public:
            float       position, velocity, current, temperature;
            uint32_t    systemTime, index;
            AK10_9(void);
            void GetFeedbackCargo(uint8_t * data);
    private:
};









extern AK10_9 hAKMotorLeftHip;
#endif
