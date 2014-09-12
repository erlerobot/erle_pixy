#include "pixycam.h"

PixyCam::PixyCam()
{
    libusb_init(&m_context);

    this->width=0;
    this->height=0;

}

Device PixyCam::getConnected()
{
    Device res = NONE;
    libusb_device_handle *handle = 0;

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

    return res;
}

int PixyCam::renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
    uint16_t x, y;
    uint32_t r, g, b;

    if(this->height==0 & this->width==0){
        this->height=height;
        this->width=width;
        mutex.lock();
        imagen.create(height, width, CV_8UC3);
        mutex.unlock();
    }
    // skip first line
    frame += width;

    mutex.lock();
    // don't render top and bottom rows, and left and rightmost columns because of color
    // interpolation
    for (y=1; y<height-1; y++)
    {
        frame++;
        for (x=1; x<width-1; x++, frame++)
        {
            interpolateBayer(width, x, y, frame, r, g, b);
            // simulate 15 bit color r >>= 4; g >>= 4; b >>= 4; r <<= 4; g <<= 4; b <<= 4;
            //*line++ = (0xFF<<24) | (r<<16) | (g<<8) | (b<<0);
            int indice = y*imagen.step+x*imagen.channels();
              imagen.data[indice]=(uchar)b;
              imagen.data[indice+1]=(uchar)g;
              imagen.data[indice+2]=(uchar)r;

        }
        frame++;
    }
    mutex.unlock();

    return 0;
}

int PixyCam::render(uint32_t type, void *args[])
{
    int res;

    // choose fourcc for representing formats fourcc.org
    if (type==FOURCC('B','A','8','1')){
        //std::cout << "renderBA81" << std::endl;
        res = renderBA81(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2], *(uint32_t *)args[3], (uint8_t *)args[4]);
    }else if (type==FOURCC('C','C','Q','1')){
        std::cout << "impossible to renderCCQ1" << std::endl;
    }else if (type==FOURCC('C', 'C', 'B', '1')){
        std::cout << "impossible to renderCCB1" << std::endl;
    }else if (type==FOURCC('C', 'M', 'V', '1')){
        std::cout << "impossible to renderCMV1" << std::endl;
    }else{ // format not recognized
        return -1;
    }
    return res;
}

void PixyCam::checkCamera()
{

    if (m_link.open()<0)
        std::cout << " runtime_error: Unable to open USB device." << std::endl;
    m_chirp = new ChirpMon(this, &m_link);

    ChirpProc versionProc;
    uint16_t *version;
    uint32_t verLen, responseInt;
    // get version and compare
    versionProc = m_chirp->getProc("version");
    if (versionProc<0){
        std::cout << "runtime_error Can't get firmware version."<< std::endl;
        exit(1);
    }
    int res = m_chirp->callSync(versionProc, END_OUT_ARGS, &responseInt, &verLen, &version, END_IN_ARGS);
    if (res<0){
        std::cout << "runtime_error Can't get firmware version."<< std::endl;
        exit(-1);
    }
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
    if (m_exec_run<0 || m_exec_running<0 || m_exec_stop<0){
        std::cout << "runtime_error Communication error with Pixy."  << std::endl;
        exit(-1);
    }
}

void PixyCam::run()
{

    int res;

    checkCamera();
    std::cout << "*** init done"<< std::endl;

    ProcInfo info;
    std::vector<std::string> m_argv;
    m_argv.push_back("runprog");
    m_argv.push_back(std::string("8"));

    // make chirp call
    int args[20];
    memset(args, 0, sizeof(args)); // zero args
    args[0]=1;
    args[1]=8;
    ChirpProc proc;

    if((proc=m_chirp->getProc(m_argv[0].c_str()))>=0 &&
        m_chirp->getProcInfo(proc, &info)>=0){
    }else{
        return;
    }


    int response;
    res = m_chirp->callSync(m_exec_run, END_OUT_ARGS, &response, END_IN_ARGS);

    if (res<0){
        exit(-1);
    }

    res = m_chirp->callAsync(proc, args[0], args[1], args[2], args[3], args[4], args[5], args[6],
                       args[7], args[8], args[9], args[10], args[11], args[12], args[13], args[14], args[15],
                       args[16], args[17], args[18], args[19], END_OUT_ARGS);
    if (res<0){
        exit(-1);
    }

    m_chirp->serviceChirp();
    // sleep a while (so we can wait for other devices to be de-registered)
    msleep(1000);
    while(1){
        dev = getConnected();
        if (dev!=NONE){
            int res, running;

            res = m_chirp->callSync(m_exec_running, END_OUT_ARGS, &running, END_IN_ARGS);
            continue;
        }
        msleep(1000);
    }
}

cv::Mat PixyCam::getImage()
{
    cv::Mat result;
    mutex.lock();
    imagen.copyTo(result);
    mutex.unlock();
    return result;
}   
