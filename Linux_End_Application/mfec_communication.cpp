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

void MFEC_USB::Tx2MFEC(uint8_t *data, uint8_t len)
{
    uint8_t txBuf[len + 8];
    txBuf[0] = 0xBB;
    txBuf[1] = 0xCC;
    txBuf[2] = len;
    memcpy(&txBuf[3], data, len);
    uint32_t crcCalculate[len + 1];
    crcCalculate[0] = (uint32_t)len;
    for (uint8_t i = 1; i <= len; i++)
    {
        crcCalculate[i] = (uint32_t)*(data + i - 1);
    }
    uint32_t crc = CRC32_32BitsInput(crcCalculate, len + 1);
    std::cout<<"tx crc is: "<<std::hex<<(unsigned int)crc<<std::endl;
    txBuf[len + 3] = (uint8_t)(crc & 0x000000FF);
    txBuf[len + 4] = (uint8_t)(crc >> 8 & 0x000000FF);
    txBuf[len + 5] = (uint8_t)(crc >> 16 & 0x000000FF);
    txBuf[len + 6] = (uint8_t)(crc >> 24 & 0x000000FF);
    txBuf[len + 7] = 0x88;

    for (uint8_t i = 0; i <= sizeof(txBuf) - 1; i++)
    {
        std::cout<<std::hex<<(unsigned int)txBuf[i]<<" ";
    }
    std::cout<<std::endl;
    MFEC_USB::serialPort.write_some(boost::asio::buffer(txBuf, len + 8));
}
