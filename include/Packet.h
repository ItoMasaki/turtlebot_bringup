#pragma once

#include <string>
#include <string.h>
#include <stdint.h>

#define PACKET_HEADER_ZERO 0xAA
#define PACKET_HEADER_ONE  0x55

namespace rt_net {
  class Translator;

  /**
   * Check Sum Error
   */
  class CheckSumError : public std::exception {
  private:
    std::string msg;
    
  public:
    CheckSumError() {
      this->msg = "CheckSumError";
    }
    
    ~CheckSumError() throw() {
    }
    
  public:
    virtual const char* what() const throw() {
      return msg.c_str();
    }
  };
  
  
  /**
   * Communication Packet
   */
  class Packet {

  private:
    uint8_t *m_pBuffer;
    uint8_t m_BufferLength;
  public:
    uint8_t *getBuffer() const {return m_pBuffer;}
    
    //uint8_t getBufferLength() const {return m_BufferLength;}
    uint8_t length() const {return m_BufferLength;}
  public:
    Packet(const uint8_t length) {
      m_pBuffer = new uint8_t[length];
      m_BufferLength = length;
    }

    Packet(const Packet& packet){
      m_pBuffer = new uint8_t[packet.length()];
      memcpy(m_pBuffer, packet.getBuffer(), packet.length());
      m_BufferLength = packet.length();
    }

    void operator=(const Packet& packet){
      m_pBuffer = new uint8_t[packet.length()];
      memcpy(m_pBuffer, packet.getBuffer(), packet.length());
      m_BufferLength = packet.length();
    }

    virtual ~Packet(){
      delete m_pBuffer;
    }

  public:
    friend class Translator;

    uint8_t& operator[](const int index) const {
      return (m_pBuffer[index]);
    }

    uint8_t  uchar(const int offset) const {
      return (m_pBuffer[offset]);
    }

    uint16_t ushort(const int offset) const {
      return m_pBuffer[offset] | (m_pBuffer[offset+1] << 8);
    }
     
    void ushort(const int offset, const uint16_t value) {
      m_pBuffer[offset  ] = value & 0xff;
      m_pBuffer[offset+1] = (value >> 8) & 0xff;
    }
  };

}
