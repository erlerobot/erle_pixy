#ifndef PIXYCAM_H
#define PIXYCAM_H

#include <iostream>

#include "usblink.h"
#include "chirpmon.h"
#include "pixy.h"
#include "helpFunctions.h"

class ChirpMon;

class PixyCam
{
public:
    PixyCam();
    void run();
    int render(uint32_t type, void *args[]);
    int renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame);

private:

    Device getConnected();
    void checkCamera();

    libusb_context *m_context;

    Device dev;
    USBLink m_link;
    ChirpMon* m_chirp;
    ChirpProc m_exec_run;
    ChirpProc m_exec_running;
    ChirpProc m_exec_stop;

    uint16_t m_version[3];
    cv::Mat imagen;
    uint16_t width;
    uint16_t height;
};

#endif // PIXYCAM_H
