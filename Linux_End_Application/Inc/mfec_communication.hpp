#ifndef MFEC_COMMUNICATION_HPP
#define MFEC_COMMUNICATION_HPP
#include <boost/asio.hpp>
#include <iostream>

class MFEC_USB
{
    public:
            boost::asio::io_service io;
            boost::asio::serial_port serialPort;
            MFEC_USB(std::string device_repo, int baudrate);
            void Communication(void);
            /*Rx message*/
            uint8_t tempRx[264];
            uint8_t msgDetectStage;
            uint8_t bytesToRead;
            uint8_t rxMessageCfrm[256];
            uint8_t rxMessageLen;
    private:

};

#endif
