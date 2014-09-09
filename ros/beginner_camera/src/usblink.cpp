//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include "usblink.h"

USBLink::USBLink()
{
    m_handle = 0;
    m_context = 0;
    m_blockSize = 64;
    m_flags = LINK_FLAG_ERROR_CORRECTED;
}

USBLink::~USBLink()
{
    if (m_handle)
        libusb_close(m_handle);
    if (m_context)
        libusb_exit(m_context);
}

int USBLink::open()
{
    libusb_init(&m_context);

    m_handle = libusb_open_device_with_vid_pid(m_context, PIXY_VID, PIXY_DID);
    if (m_handle==NULL){
        return -1;
    }
    if (libusb_set_configuration(m_handle, 1)<0){
        libusb_close(m_handle);
        m_handle = 0;
        return -1;
    }
    if (libusb_claim_interface(m_handle, 1)<0){
        libusb_close(m_handle);
        m_handle = 0;
        return -1;
    }

    libusb_reset_device(m_handle);

    return 0;
}


int USBLink::send(const uint8_t *data, uint32_t len, uint16_t timeoutMs)
{
    int res, transferred;

    if (timeoutMs==0) // 0 equals infinity
        timeoutMs = 10;

    if ((res=libusb_bulk_transfer(m_handle, 0x02, (unsigned char *)data, len, &transferred, timeoutMs))<0)
    {
        if(res==LIBUSB_ERROR_TIMEOUT){
            std::cout << "LIBUSB_ERROR_TIMEOUT:" << std::endl;
        }
        if(res==LIBUSB_ERROR_PIPE){
            std::cout << "LIBUSB_ERROR_PIPE:" << std::endl;
        }
        if(res==LIBUSB_ERROR_OVERFLOW ){
            std::cout << "LIBUSB_ERROR_OVERFLOW :" << std::endl;
        }
        if(res==LIBUSB_ERROR_NO_DEVICE){
            std::cout << "LIBUSB_ERROR_NO_DEVICE:" << std::endl;
        }
        std::cout << "libusb_bulk_write "<< res << std::endl; ;
        return res;
    }
    return transferred;
}

int USBLink::receive(uint8_t *data, uint32_t len, uint16_t timeoutMs)
{
    int res, transferred;

    if (timeoutMs==0) // 0 equals infinity
        timeoutMs = 50;

    if ((res=libusb_bulk_transfer(m_handle, 0x82, (unsigned char *)data, len, &transferred, timeoutMs))<0)
    {
        std::cout << "libusb_bulk_read "<< res << std::endl; ;
        return res;
    }
    return transferred;
}

void USBLink::setTimer()
{
}

uint32_t USBLink::getTimer()
{
    return 1;
}



