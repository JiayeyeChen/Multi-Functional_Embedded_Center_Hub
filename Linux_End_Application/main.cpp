#include <iostream>
#include <boost/asio.hpp>
#include <main.hpp>
#include "mfec_communication.hpp"

int main(int argc, char **argv)
{
    MFEC_USB myMFEC_USB("/dev/ttyACM0", 115200);
    
    while (1)
    {
        myMFEC_USB.Communication();
    }
    // boost::asio::io_service myIO;

    // boost::asio::serial_port MFECSerialPort(myIO, "/dev/ttyACM0");

    // if (MFECSerialPort.is_open())
    //     MFECSerialPort.close();
    // MFECSerialPort.open("/dev/ttyACM0");
    // MFECSerialPort.set_option(boost::asio::serial_port_base::baud_rate(115200));
    // MFECSerialPort.set_option(boost::asio::serial_port::flow_control());
    // MFECSerialPort.set_option(boost::asio::serial_port::parity());
    // MFECSerialPort.set_option(boost::asio::serial_port::stop_bits());
    // MFECSerialPort.set_option(boost::asio::serial_port::character_size(8));

    // if (MFECSerialPort.is_open())
    // {
    //     std::cout << "Serial port opened successfully!" << std::endl;
    // }

    // std::cout << "Goodluck Jiaye" << std::endl;

    // for (;;)
    // {
    //     uint8_t data[256];
    //     std::string input;
    //     size_t nread = boost::asio::read(MFECSerialPort, boost::asio::buffer(data, 1));
    //     std::cout.fill('0');
    //     std::cout.width(2);
    //     std::cout << std::hex << (int)data[0] << std::endl;
    // }
}