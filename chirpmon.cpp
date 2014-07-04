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

#include <QDebug>
#include <QMutexLocker>
#include <stdexcept>
#include "chirpmon.h"
#include <iostream>
//#include "interpreter.h"
struct Frame8
{
    Frame8()
    {
        m_pixels = (uint8_t *)NULL;
        m_width = m_height = 0;
    }

    Frame8(uint8_t *pixels, uint16_t width, uint16_t height)
    {
        m_pixels = pixels;
        m_width = width;
        m_height = height;
    }

    uint8_t *m_pixels;
    int16_t m_width;
    int16_t m_height;
};
Frame8 m_rawFrame;

inline void interpolateBayer(unsigned int width, unsigned int x, unsigned int y, unsigned char *pixel, unsigned int &r, unsigned int &g, unsigned int &b)
{
#if 1
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = (*(pixel-width)+*(pixel+width))>>1;
        }
    }
    else
    {
        if (x&1)
        {
            r = (*(pixel-width)+*(pixel+width))>>1;
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
            g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = (*(pixel-width-1)+*(pixel-width+1))>>1;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel+width);
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel+width-1)+*(pixel+width+1))>>1;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = (*(pixel-width-1)+*(pixel-width+1))>>1;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel-width);
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel-width-1)+*(pixel-width+1))>>1;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = *(pixel-1);
            b = *(pixel-width-1);
        }
        else
        {
            r = *(pixel-1);
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel-width);
            g = *pixel;
            b = *(pixel-1);
        }
        else
        {
            r = *(pixel-width-1);
            g = *(pixel-1);
            b = *pixel;
        }
    }
#endif
}


ChirpMon::ChirpMon(/*Interpreter *interpreter,*/ USBLink *link) : m_mutex(QMutex::Recursive)
{
    m_hinterested = true;
    m_client = true;
//    m_interpreter = interpreter;

    if (setLink((Link*)link)<0)
        //std::cout << "ChirpMon: runtime_error Unable to connect to device." <<std::endl;
        throw std::runtime_error("Unable to connect to device.");
    m_rawFrame.m_pixels = new uint8_t[0x10000];

}

ChirpMon::~ChirpMon()
{
}


int ChirpMon::serviceChirp()
{
    uint8_t type;
    ChirpProc recvProc;
    void *args[CRP_MAX_ARGS+1];
    int res;

    while(1)
    {
        if ((res=recvChirp(&type, &recvProc, args, true))<0)
            return res;
        handleChirp(type, recvProc, args);
        if (type&CRP_RESPONSE)
            break;
    }
    return 0;
}

QString printType(uint32_t val)
{
    bool parens=false;
    QString res;
    QChar a, b, c, d;
    uint32_t val2 = val;

    a = (QChar)(val2&0xff);
    val2 >>= 8;
    b = (QChar)(val2&0xff);
    val2 >>= 8;
    c = (QChar)(val2&0xff);
    val2 >>= 8;
    d = (QChar)(val2&0xff);

    if (a.isPrint() && b.isPrint() && c.isPrint() && d.isPrint())
    {
        if (parens)
            res = QString("FOURCC(") + a + b + c + d + ")";
        else
            res = QString(a) + b + c + d;
    }
    else
    {
        if (parens)
            res = "HTYPE(0x" + QString::number((uint)val, 16) + ")";
        else
            res = "0x" + QString::number((uint)val, 16);
    }

    return res;
}

int ChirpMon::handleChirp(uint8_t type, ChirpProc proc, void *args[])
{
    if (type==CRP_RESPONSE)
    {
        // strip off response, add to print string
        //    m_print = "response " + QString::number(m_rcount++) + ": " +
        m_print = "response: " +
                QString::number(*(int *)args[0]) + " (0x" + QString::number((uint)*(uint *)args[0], 16) + ") ";

        // render rest of response, if present
        handleData(args+1);

        return 0;
    }

    return Chirp::handleChirp(type, proc, args);
}

int renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
    uint16_t x, y;
    uint32_t *line;
    uint32_t r, g, b;

    memcpy(m_rawFrame.m_pixels, frame, width*height);
    m_rawFrame.m_width = width;
    m_rawFrame.m_height = height;

    // skip first line
    frame += width;

    // don't render top and bottom rows, and left and rightmost columns because of color
    // interpolation
    QImage img(width-2, height-2, QImage::Format_RGB32);
    cv::Mat imagen;
    imagen.create(height, width, CV_8UC3);

    for (y=1; y<height-1; y++)
    {
        line = (unsigned int *)img.scanLine(y-1);
        frame++;
        for (x=1; x<width-1; x++, frame++)
        {
            interpolateBayer(width, x, y, frame, r, g, b);
            // simulate 15 bit color r >>= 4; g >>= 4; b >>= 4; r <<= 4; g <<= 4; b <<= 4;
            *line++ = (0x40<<24) | (r<<16) | (g<<8) | (b<<0);
            int indice = y*imagen.step+x*imagen.channels();
              imagen.data[indice]=(u_int8_t)b;
              imagen.data[indice+1]=(u_int8_t)g;
              imagen.data[indice+2]=(u_int8_t)g;

        }
        frame++;
    }

    cv::imshow("imagen", imagen);
    cv::waitKey(20);
    img.save("image.jpg");


    // send image to ourselves across threads
    // from chirp thread to gui thread
//    emitImage(img);

  //  m_background = img;

    //if (renderFlags&RENDER_FLAG_FLUSH)
    //    emitFlushImage();

    return 0;
}

int render(uint32_t type, void *args[])
{
    int res;

    // choose fourcc for representing formats fourcc.org
    if (type==FOURCC('B','A','8','1')){
        std::cout << "renderBA81" << std::endl;
        res = renderBA81(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2], *(uint32_t *)args[3], (uint8_t *)args[4]);
    }else if (type==FOURCC('C','C','Q','1')){
        std::cout << "renderCCQ1" << std::endl;
    //    res = renderCCQ1(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2], *(uint32_t *)args[3], (uint32_t *)args[4]);
    }else if (type==FOURCC('C', 'C', 'B', '1')){
        std::cout << "renderCCB1" << std::endl;
    }else if (type==FOURCC('C', 'M', 'V', '1')){
        std::cout << "renderCMV1" << std::endl;
//        res = renderCMV1(*(uint8_t *)args[0], *(uint32_t *)args[1], (float *)args[2], *(uint16_t *)args[3], *(uint32_t *)args[4], *(uint32_t *)args[5], (uint8_t *)args[6]);
    }else{ // format not recognized
        return -1;
    }
    return res;
}
void ChirpMon::handleData(void *args[])
{
    uint8_t type;
    if (args[0])
    {
        type = Chirp::getType(args[0]);
        if (type==CRP_TYPE_HINT)
        {
            m_print += printType(*(uint32_t *)args[0]) + " frame data\n";
            render(*(uint32_t *)args[0], args+1);
        }
        else if (type==CRP_HSTRING)
        {
            m_print +=  (char *)args[0];
        }
        else
            qDebug() << "unknown type " << type;
    }
    if (m_print.right(1)!="\n")
        m_print += "\n";
}


void ChirpMon::handleXdata(void *data[])
{
    handleData(data);
}

int ChirpMon::sendChirp(uint8_t type, ChirpProc proc)
{   // this is only called when we call call()
    int res;

    // if we're programming (defining the program), put all the calls in m_program
    // otherwise pass the call the Chirp::sendChirp() so it gets sent out.
    // todo: save the call and use the chirp thread to send (so send and receive are handled by
    // same thread. not sure how important that is...)
 /*   if (m_interpreter->m_programming && !(type&CRP_INTRINSIC))
    {
        // put on queue
        // only copy data (not header).  Header hasn't been written to buffer yet.
        m_interpreter->addProgram(ChirpCallData(type, proc, m_buf+m_headerLen, m_len));
        return 0;
    }
*/
    res = Chirp::sendChirp(type, proc);

    return res;

}

int ChirpMon::execute(const ChirpCallData &data)
{
    QMutexLocker locker(&m_mutex);
    int res;

    // copy into chirp buffer-- remember to skip the header space
    memcpy(m_buf+m_headerLen, data.m_buf, data.m_len);
    m_len = data.m_len;
    if ((res=Chirp::sendChirp(data.m_type, data.m_proc))<0)
        return res;
    if ((res=serviceChirp())<0)
        return res;

    return 0;
}

