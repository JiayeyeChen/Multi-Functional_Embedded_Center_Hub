#include "mfec_communication.hpp"
#include <boost/asio.hpp>
#include "crc32_mpeg.hpp"

MFEC_USB::MFEC_USB(std::string device_repo, int baudrate) : serialPort(io, device_repo), msgDetectStage(0)
{
    if (!MFEC_USB::serialPort.is_open())
    {
        try
        {
            MFEC_USB::serialPort.open(device_repo);
        }
        catch (...) // Problem: Cannot catch exception when USB is disconnected
        {
            std::cout << "Multi-FUnctional Embedded Center PC End Serial Port Opening Failed." << std::endl;
        }
    }
    if (MFEC_USB::serialPort.is_open())
    {
        MFEC_USB::serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        MFEC_USB::serialPort.set_option(boost::asio::serial_port::flow_control());
        MFEC_USB::serialPort.set_option(boost::asio::serial_port::parity());
        MFEC_USB::serialPort.set_option(boost::asio::serial_port::stop_bits());
        MFEC_USB::serialPort.set_option(boost::asio::serial_port::character_size(8));
        std::cout << "Multi-FUnctional Embedded Center PC End Serial Port Opening Successful." << std::endl;
    }
    else
    {
        std::cout << "Multi-FUnctional Embedded Center PC End Serial Port Opening Failed." << std::endl;
    }
}

void MFEC_USB::Communication(void)
{
    if (msgDetectStage < 3)
    {
        if (boost::asio::read(MFEC_USB::serialPort, boost::asio::buffer(MFEC_USB::tempRx, 1)))
        {
            if (msgDetectStage == 0)
            {
                if (tempRx[0] == 0xAA)
                    msgDetectStage = 1;
            }
            else if (msgDetectStage == 1)
            {
                if (tempRx[0] == 0xCC)
                    msgDetectStage = 2;
                else
                    msgDetectStage = 0;
            }
            else if (msgDetectStage == 2)
            {
                bytesToRead = tempRx[0];
                msgDetectStage = 3;
            }
        }
    }
    else if (msgDetectStage == 3)
    {
        if (boost::asio::read(MFEC_USB::serialPort, boost::asio::buffer(MFEC_USB::tempRx, bytesToRead + 5)))
        {
            // std::cout << "Reached crc check stage." << std::endl;
            uint32_t crcCalculate[bytesToRead + 1];
            crcCalculate[0] = (uint32_t)bytesToRead;
            // std::cout << std::hex << (int)crcCalculate[0] << std::endl;////////////////////
            for (uint8_t i = 0; i <= bytesToRead - 1; i++)
            {
                crcCalculate[i + 1] = (uint32_t)tempRx[i];
                // std::cout << std::hex << (int)crcCalculate[i + 1] << std::endl;////////////////////
            }
            uint32_t crcResult = CRC32_32BitsInput(crcCalculate, bytesToRead + 1);
            // std::cout << "CRC calculated result: " << std::hex << (unsigned int)crcResult << std::endl;
            uint32_t crcFromMFEC = tempRx[bytesToRead] | tempRx[bytesToRead + 1] << 8 | tempRx[bytesToRead + 2] << 16 | tempRx[bytesToRead + 3] << 24;


            if (crcResult == crcFromMFEC && tempRx[bytesToRead + 4] == 0x55)
            {
                msgDetectStage = 0;
                std::cout<<"Cargo Received!"<<std::endl;
                memcpy(rxMessageCfrm, tempRx,bytesToRead);
                rxMessageLen = bytesToRead;

                // std::cout<<"Message is :" << std::endl;
                // for (uint8_t i = 0; i <= bytesToRead - 1; i++)
                // {
                //     std::cout.fill('0');
                //     std::cout.width(2);
                //     std::cout << std::hex<< (int)rxMessageCfrm[i] << " ";
                // }
                // std::cout << std::endl;
            }
            else
            {
                std::cout<<"Invalid Cargo!"<<std::endl;
            }

            // std::cout << "CRC real result: " << std::hex << (unsigned int)crcFromMFEC << std::endl;
        }
    }
}
