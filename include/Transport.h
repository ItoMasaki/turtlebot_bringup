#pragma once
#include <exception>
#include "SerialPort.h"
#include "Packet.h"

namespace rt_net {

  class TimeOutException : std::exception {
  private:
    std::string msg;
    
  public:
    TimeOutException(const char* msg) {
      this->msg = "TimeOutException: ";
      this->msg += msg;
    }

    TimeOutException() {
      this->msg = "TimeOutException";
    }
    
    ~TimeOutException() throw() {
    }
    
  public:
    virtual const char* what() const throw() {
      return msg.c_str();
    }
  };


  class Transport {
  private:
    net::ysuga::SerialPort* m_pSerialPort;
    
  public:
    Transport(net::ysuga::SerialPort* pSerialPort);
    virtual ~Transport();
    
    Packet receive();
    void transmit(const Packet& packet);
  };
  
}
