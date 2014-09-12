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
#include <stdexcept>
#include "chirpmon.h"
#include <iostream>

ChirpMon::ChirpMon(PixyCam* pixycam, USBLink *link)
{
    m_hinterested = true;
    m_client = true;

    if (setLink((Link*)link)<0)
        throw std::runtime_error("Unable to connect to device.");

    // inicialize memory
    this->pixycam = pixycam;
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

    while(1){
        if ((res=recvChirp(&type, &recvProc, args, true))<0)
            return res;
        handleChirp(type, recvProc, args);
        if (type&CRP_RESPONSE)
            break;
    }
    return 0;
}

std::string printType(uint32_t val)
{
    bool parens=false;
    std::string res_s;
    char a_c, b_c, c_c, d_c;
    uint32_t val2 = val;
    a_c = (char)(val2&0xff);
    val2 >>= 8;
    b_c = (char)(val2&0xff);
    val2 >>= 8;
    c_c = (char)(val2&0xff);
    val2 >>= 8;
    d_c = (char)(val2&0xff);


    if (true){
        if (parens){
            res_s = std::string("FOURCC(");
            res_s.push_back(a_c);
            res_s.push_back(b_c);
            res_s.push_back(c_c);
            res_s.push_back(d_c);
            res_s+=")";
        }else{
            res_s.push_back(a_c);
            res_s.push_back(b_c);
            res_s.push_back(c_c);
            res_s.push_back(d_c);
        }
    }else{
        std::stringstream stream;
        stream << std::hex << val;
        std::string result( stream.str() );

        if (parens){
            res_s = "HTYPE(0x" + result + ")";
        }else{
            res_s = "0x" + result;
        }
    }

    return res_s;
}

int ChirpMon::handleChirp(uint8_t type, ChirpProc proc, void *args[])
{
    if (type==CRP_RESPONSE)
    {
        // strip off response, add to print string
        //    m_print = "response " + QString::number(m_rcount++) + ": " +

        std::stringstream stream;
        stream << *(int *)args[0];
        stream << " (0x";
        stream << std::hex << (uint)*(uint *)args[0], 16;
        stream << ")";

        m_print = stream.str();

        // render rest of response, if present
        handleData(args+1);

        return 0;
    }

    return Chirp::handleChirp(type, proc, args);
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
            pixycam->render(*(uint32_t *)args[0], args+1);
        }
        else if (type==CRP_HSTRING)
        {
            m_print +=  (char *)args[0];
        }
        else
            std::cout <<"unknown type " << type << std::endl;
    }
    if (m_print[m_print.size()-1]!= '\n')
        m_print += "\n";
}


void ChirpMon::handleXdata(void *data[])
{
    handleData(data);
}

int ChirpMon::sendChirp(uint8_t type, ChirpProc proc)
{
    // this is only called when we call call()
    int res;

    res = Chirp::sendChirp(type, proc);

    return res;

}

int ChirpMon::execute(const ChirpCallData &data)
{
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

