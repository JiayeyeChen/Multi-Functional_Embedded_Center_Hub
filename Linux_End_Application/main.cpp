#include <iostream>
#include <boost/asio.hpp>
#include <main.hpp>
#include "mfec_communication.hpp"
#include "crc32_mpeg.hpp"
#include <chrono>
#include <thread>
#include "ak10_9_calibration.hpp"

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    // MFEC_USB myMFEC_USB("/dev/ttyACM0", 921600);
    while (1)
    {
        myMFEC_USB.Communication();

        if (myMFEC_USB.ifNewMessage)
        {
            hAKMotorLeftHip.GetFeedbackCargo(myMFEC_USB.rxMessageCfrm);
        }
        // uint8_t txMsg[2];
        // txMsg[0] = 0x01;
        // txMsg[1] = 0x02;
        // myMFEC_USB.Tx2MFEC(txMsg, 2);

        // std::this_thread::sleep_for(1000ms);
    }
}
