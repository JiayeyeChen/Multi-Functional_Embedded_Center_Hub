#include "mfec_communication.hpp"
#include <boost/asio.hpp>

MFEC_USB::MFEC_USB(std::string device_repo, int baudrate) : serialPort(io, device_repo)
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
    if (boost::asio::read(MFEC_USB::serialPort, boost::asio::buffer(MFEC_USB::tempRx, 1)))
    {
        std::cout.fill('0');
        std::cout.width(2);
        std::cout << std::hex <<(int)tempRx[0] << std::endl;
    }
}
