#include <iostream>
#include <string.h>
#include <stdio.h>

#include "usblink.h"
#include "chirpmon.h"
#include "pixy.h"

libusb_context *m_context;
QMutex m_mutex;

enum Device {NONE, PIXY, PIXY_DFU};
#define VER_MAJOR       0
#define VER_MINOR       1
#define VER_BUILD       49

Device dev;
USBLink m_link;
ChirpMon* m_chirp;
ChirpProc m_exec_run;
ChirpProc m_exec_running;
ChirpProc m_exec_stop;
bool m_notified = false;

Device getConnected()
{
    Device res = NONE;
    libusb_device_handle *handle = 0;

    m_mutex.lock();
    handle = libusb_open_device_with_vid_pid(m_context, PIXY_VID, PIXY_DID);
    if (handle)
        res = PIXY;
    else
    {
        handle = libusb_open_device_with_vid_pid(m_context, PIXY_DFU_VID, PIXY_DFU_DID);
        if (handle)
            res = PIXY_DFU;
    }
    if (handle)
        libusb_close(handle);
    m_mutex.unlock();

    return res;
}
#include <sys/time.h>
void msleep (unsigned int ms) {
    int microsecs;
    struct timeval tv;
    microsecs = ms * 1000;
    tv.tv_sec  = microsecs / 1000000;
    tv.tv_usec = microsecs % 1000000;
    select (0, NULL, NULL, NULL, &tv);
}

int main(int argc, char* argv[])
{

    libusb_init(&m_context);


    uint16_t m_version[3];

//    connect(this, SIGNAL(connected(Device,bool)), m_main, SLOT(handleConnected(Device,bool)));

    dev = getConnected();

    if (m_link.open()<0)
        std::cout << " runtime_error: Unable to open USB device." << std::endl;
    m_chirp = new ChirpMon(&m_link);

    ChirpProc versionProc;
    uint16_t *version;
    uint32_t verLen, responseInt;
    // get version and compare
    versionProc = m_chirp->getProc("version");
    if (versionProc<0)
        std::cout << "runtime_error Can't get firmware version."<< std::endl;
    int res = m_chirp->callSync(versionProc, END_OUT_ARGS, &responseInt, &verLen, &version, END_IN_ARGS);
    if (res<0)
        std::cout << "runtime_error Can't get firmware version."<< std::endl;
    memcpy(m_version, version, 3*sizeof(uint16_t));
    if (m_version[0]!=VER_MAJOR || m_version[1]>VER_MINOR)
    {
        char buf[0x100];
        sprintf(buf, "This Pixy's firmware version (%d.%d.%d) is not compatible with this PixyMon version (%d.%d.%d).",
                m_version[0], m_version[1], m_version[2], VER_MAJOR, VER_MINOR, VER_BUILD);
        std::cout << (buf) << std::endl;
    }


    m_exec_run = m_chirp->getProc("run");
    m_exec_running = m_chirp->getProc("running");
    m_exec_stop = m_chirp->getProc("stop");
    if (m_exec_run<0 || m_exec_running<0 || m_exec_stop<0)
        std::cout << "runtime_error Communication error with Pixy."  << std::endl;
    std::cout << "*** init done"<< std::endl;

    ProcInfo info;

    QStringList m_argv;
    m_argv.append("runprog");
    m_argv.append("8");

    // make chirp call
    int args[20];
    memset(args, 0, sizeof(args)); // zero args
    args[0]=1;
    args[1]=8;
    ChirpProc proc;

    if((proc=m_chirp->getProc(m_argv[0].toLocal8Bit()))>=0 &&
        m_chirp->getProcInfo(proc, &info)>=0)
    {

    }else{
        return 25;
    }


    int response;
    res = m_chirp->callSync(m_exec_run, END_OUT_ARGS, &response, END_IN_ARGS);

    res = m_chirp->callAsync(proc, args[0], args[1], args[2], args[3], args[4], args[5], args[6],
                       args[7], args[8], args[9], args[10], args[11], args[12], args[13], args[14], args[15],
                       args[16], args[17], args[18], args[19], END_OUT_ARGS);

    m_chirp->serviceChirp();
    // sleep a while (so we can wait for other devices to be de-registered)
    msleep(1000);
    while(1)
    {
        dev = getConnected();
        if (dev!=NONE)
        {
            int res, running;

            res = m_chirp->callSync(m_exec_running, END_OUT_ARGS, &running, END_IN_ARGS);
            qDebug("running %d %d", res, running);


            continue;
        }
        msleep(1000);
    }

}
