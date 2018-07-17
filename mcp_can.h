#ifndef _MCP2515_H_
#define _MCP2515_H_

#undef rtr

#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{

    private:

    byte   ext_flg;                         // identifier xxxID
                                            // either extended (the 29 LSB) or standard (the 11 LSB)
    unsigned long  can_id;                  // can id
    byte   rtr;                             // rtr
    byte   SPICS;
    SPIClass *pSPI;
    byte   nReservedTx;                     // Count of tx buffers for reserved send


private:

    void mcp2515_reset(void);                                   // reset mcp2515

    byte mcp2515_readRegister(const byte address);              // read mcp2515's register

    void mcp2515_readRegisterS(const byte address,
	                       byte values[],
                               const byte n);
    void mcp2515_setRegister(const byte address,                // set mcp2515's register
                             const byte value);

    void mcp2515_setRegisterS(const byte address,               // set mcp2515's registers
                              const byte values[],
                              const byte n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const byte address,             // set bit of one register
                                const byte mask,
                                const byte data);

    byte mcp2515_readStatus(void);                              // read mcp2515's Status
    byte mcp2515_setCANCTRL_Mode(const byte newmode);           // set mode
    byte mcp2515_configRate(const byte canSpeed, const byte clock);  // set baudrate





    byte mcp2515_init(const byte canSpeed, const byte clock);   // mcp2515init

    void mcp2515_write_id( const byte mcp_addr,                 // write can id
                               const byte ext,
                               const unsigned long id );
    void mcp2515_read_id( const byte mcp_addr,                  // read can id
                                    byte* ext,
                                    unsigned long* id );
    void mcp2515_write_canMsg( const byte buffer_sidh_addr, unsigned long id, byte ext, byte rtr, byte len, volatile const byte *buf);     // read can msg
    void mcp2515_start_transmit(const byte mcp_addr);           // start transmit
    byte mcp2515_getNextFreeTXBuf(byte *txbuf_n);               // get Next free txbuf
    byte mcp2515_isTXBufFree(byte *txbuf_n, byte iBuf);         // is buffer by index free
    void mcp2515_read_canMsg( const byte buffer_load_addr, volatile unsigned long *id, volatile byte *ext, volatile byte *rtr, volatile byte *len, volatile byte *buf);   // write can msg
    byte sendMsg(unsigned long id, byte extended, byte rtrBit, byte len, const byte *buf, bool wait_sent=true); // send message

    uint32_t filter_filters[6] = { 0 };
    bool filter_ext_bit[6] = { 0 };

    uint32_t filter_masks[2] = { 0 };
    bool mask_ext_bit[2] = { 0 };


public:
    MCP_CAN(byte _CS=0);
    void init_CS(byte _CS);                      // define CS after construction before begin()
    void enableTxInterrupt(bool enable=true);    // enable transmit interrupt
    byte begin(uint32_t speedset, const byte clockset = MCP_16MHz);     // init can
    byte sendMsgBuf(unsigned long id, byte extended, byte len, const byte *buf, bool wait_sent=true);               // send buf
    byte sendMsgBuf(unsigned long id, byte extended, byte rtrBit, byte len, const byte *buf, bool wait_sent=true);  // send buf
    byte sendMsgBuf(byte status, unsigned long id, byte extended, byte rtrBit, byte len, volatile const byte *buf); // send message buf by using parsed buffer status
    byte checkReceive(void);                                        // if something received
    byte readMsgBuf(byte *len, byte *buf);                          // read buf
    byte readMsgBufID(unsigned long *ID, byte *len, byte *buf);     // read buf with object ID
    byte readMsgBufID(byte status, volatile unsigned long *id, volatile byte *extended, volatile byte *remote, volatile byte *len, volatile byte *buf); // read buf with object ID
    byte readRxTxStatus(void);                                      // read has something send or received
    void clearBufferTransmitIfFlags(byte flags=0);                  // Clear transmit flags according to status
    byte checkClearRxStatus(byte *status);                          // read and clear and return first found rx status bit
    byte checkClearTxStatus(byte *status, byte iTxBuf=0xff);        // read and clear and return first found or buffer specified tx status bit
    byte checkError(void);                                          // if something error
    unsigned long getCanId(void);                                   // get can id when receive
    byte isRemoteRequest(void);                                     // get RR flag when receive
    byte isExtendedFrame(void);                                     // did we recieve 29bit frame?
    byte trySendMsgBuf(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf, byte iTxBuf=0xff);  // as sendMsgBuf, but does not have any wait for free buffer
    inline byte trySendExtMsgBuf(unsigned long id, byte len, const byte *buf, byte iTxBuf=0xff) {  // as trySendMsgBuf, but set ext=1 and rtr=0
      return trySendMsgBuf(id,1,0,len,buf,iTxBuf);
    }
    byte init_Mask(byte num, byte ext, unsigned long ulData);       // init Masks
    byte init_Filt(byte num, byte ext, unsigned long ulData);       // init filters
    inline byte sendExtMsgBuf(byte status, unsigned long id, byte len, volatile const byte *buf) { // as sendMsgBuf, but set ext=1 and rtr=0
      return sendMsgBuf(status,id,1,0,len,buf);
    }

};

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
