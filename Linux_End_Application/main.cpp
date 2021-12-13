#include <iostream>
#include <boost/asio.hpp>
#include <main.hpp>
#include "mfec_communication.hpp"
#include "crc32_mpeg.hpp"
#include <chrono>
#include <thread>
#include "ak10_9_calibration.hpp"
#include <fstream>

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    std::ofstream fileStream;
    
    
    uint8_t ifDatalogStarted = 0;

    // uint8_t test[5] = {0x41, 0x42, 0x43, 0x44, 0x45};
    // uint8_t* testptr = test;
    // std::string testStr = (char*)testptr;
    // std::cout<<testStr<<std::endl;
    while (1)
    {
        myMFEC_USB.Communication();

        if (myMFEC_USB.ifNewMessage)
        {
            if (ifDatalogStarted)
            {
                if (myMFEC_USB.rxMessageLen == 12)
                {
                    char getMsg[myMFEC_USB.rxMessageLen];
                    memcpy(getMsg, myMFEC_USB.rxMessageCfrm, myMFEC_USB.rxMessageLen);
                    std::string detectEndMsg = getMsg;
                    if (detectEndMsg == "Datalog end")
                    {
                        ifDatalogStarted = 0;
                        std::cout << detectEndMsg << std::endl;
                        fileStream.close();
                    }
                }
                else
                {
                     hAKMotorLeftHip.GetFeedbackCargo(myMFEC_USB.rxMessageCfrm);
                     fileStream << hAKMotorLeftHip.index << "," << hAKMotorLeftHip.systemTime << "," << hAKMotorLeftHip.position << "," << hAKMotorLeftHip.velocity << "," << hAKMotorLeftHip.current << std::endl;
                     
                }
            }
            else
            {
                char getMsg[myMFEC_USB.rxMessageLen];
                memcpy(getMsg, myMFEC_USB.rxMessageCfrm, myMFEC_USB.rxMessageLen);
                std::string detectStartMsg = getMsg;
                
                if (detectStartMsg == "Datalog start")
                {
                    ifDatalogStarted = 1;
                    std::cout << detectStartMsg << std::endl;
                    fileStream.open("AK10-9 Datalog.csv");
                    fileStream << "Index,Time (ms),Position,Velocity,Iq,Temperature" << std::endl;
                    fileStream << "test" << std::endl;
                }
            }

            myMFEC_USB.ifNewMessage = 0;
        }

        // std::this_thread::sleep_for(1000ms);
    }
}
