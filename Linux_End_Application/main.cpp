#include <iostream>
#include <boost/asio.hpp>
#include <main.hpp>
#include "mfec_communication.hpp"
#include "crc32_mpeg.hpp"

int main(int argc, char **argv)
{
    MFEC_USB myMFEC_USB("/dev/ttyACM0", 115200);
    while (1)
    {
        myMFEC_USB.Communication();
    }
}