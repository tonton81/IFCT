#include "mcp_can.h"
#include <IFCT.h>
#include <kinetis_flexcan.h>

#undef rtr

CAN_message_t storage_mcp_sim;


/*********************************************************************************************************
** Function name:           MCP_CAN
** Descriptions:            Constructor
*********************************************************************************************************/
MCP_CAN::MCP_CAN(byte _CS) : nReservedTx(0)
{
}

/*********************************************************************************************************
** Function name:           set CS
** Descriptions:            init CS pin and set UNSELECTED
*********************************************************************************************************/
void MCP_CAN::init_CS(byte _CS)
{
}

/*********************************************************************************************************
** Function name:           begin
** Descriptions:            init can and set speed
*********************************************************************************************************/
byte MCP_CAN::begin(uint32_t speedset, const byte clockset)
{
    Can0.setBaudRate(speedset);
    return CAN_OK;
}


/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
byte MCP_CAN::sendMsg(unsigned long id, byte extended, byte rtrBit, byte len, const byte *buf, bool wait_sent)
{
#define rtr flags.remote
    CAN_message_t frame;
    ( rtrBit ) ? frame.rtr = 1 : frame.rtr = 0;
    ( extended ) ? frame.ext = 1 : frame.ext = 0;
    frame.id = id;
    frame.len = len;
    memmove(&frame.buf[0],&buf[0],8);
    Can0.write(frame);
    return CAN_OK;
#undef rtr
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
byte MCP_CAN::sendMsgBuf(unsigned long id, byte extended, byte rtrBit, byte len, const byte *buf, bool wait_sent)
{
    return sendMsg(id,extended,rtrBit,len,buf,wait_sent);
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
byte MCP_CAN::sendMsgBuf(unsigned long id, byte extended, byte len, const byte *buf, bool wait_sent)
{
    return sendMsg(id,extended,0,len,buf,wait_sent);
}


/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
byte MCP_CAN::checkReceive(void)
{
    if ( Can0.read(storage_mcp_sim) ) {
      if ( !filter_masks[0] || !filter_masks[1] ) return CAN_MSGAVAIL; /* not all masks set */
      for ( uint8_t i = 0; i < 6; i++ ) {
        if ( !filter_filters[i] ) return CAN_MSGAVAIL;  /* not all filters set */
        if ( (( filter_masks[0] & storage_mcp_sim.id ) == filter_filters[i]) || (( filter_masks[1] & storage_mcp_sim.id ) == filter_filters[i])  ) {
          if ( filter_ext_bit[i] == storage_mcp_sim.ext ) return CAN_MSGAVAIL;
          return CAN_NOMSG;
        }
      }
    }
    return CAN_NOMSG;
}


/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message buf
*********************************************************************************************************/
byte MCP_CAN::readMsgBuf(byte *len, byte buf[])
{
    return readMsgBufID(readRxTxStatus(),&can_id,&ext_flg,&rtr,len,buf);
}



/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            read message buf and can bus source ID
*********************************************************************************************************/
byte MCP_CAN::readMsgBufID(unsigned long *ID, byte *len, byte buf[])
{
    return readMsgBufID(readRxTxStatus(),ID,&ext_flg,&rtr,len,buf);
}


/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            Read message buf and can bus source ID according to status.
**                          Status has to be read with readRxTxStatus.
*********************************************************************************************************/
byte MCP_CAN::readMsgBufID(byte status, volatile unsigned long *id, volatile byte *extended, volatile byte *rtrBit, volatile byte *len, volatile byte *buf)
{
    mcp2515_read_canMsg( MCP_READ_RX0, id, extended, rtrBit, len, buf);
}



/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/

#undef rtr
#undef ext
void MCP_CAN::mcp2515_read_canMsg( const byte buffer_load_addr, volatile unsigned long *id, volatile byte *ext, volatile byte *rtrBit, volatile byte *len, volatile byte *buf)        /* read can msg                 */
{

    *id = storage_mcp_sim.id;
    *ext = storage_mcp_sim.flags.extended;
    *len = storage_mcp_sim.len;
    *rtrBit = storage_mcp_sim.flags.remote;
    for ( uint8_t i = 0; i < 8; i++ ) buf[i] = storage_mcp_sim.buf[i];
}
#define rtr flags.remote
#define ext flags.extended





/*********************************************************************************************************
** Function name:           readRxTxStatus
** Descriptions:            Read RX and TX interrupt bits. Function uses status reading, but translates.
**                          result to MCP_CANINTF. With this you can check status e.g. on interrupt sr
**                          with one single call to save SPI calls. Then use checkClearRxStatus and
**                          checkClearTxStatus for testing.
*********************************************************************************************************/
byte MCP_CAN::readRxTxStatus(void)
{
}



/*********************************************************************************************************
** Function name:           checkClearRxStatus
** Descriptions:            Return first found rx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
byte MCP_CAN::checkClearRxStatus(byte *status)
{
}

/*********************************************************************************************************
** Function name:           checkClearTxStatus
** Descriptions:            Return specified buffer of first found tx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
byte MCP_CAN::checkClearTxStatus(byte *status, byte iTxBuf)
{
}

/*********************************************************************************************************
** Function name:           clearBufferTransmitIfFlags
** Descriptions:            Clear transmit interrupt flags for specific buffer or for all unreserved buffers.
**                          If interrupt will be used, it is important to clear all flags, when there is no
**                          more data to be sent. Otherwise IRQ will newer change state.
*********************************************************************************************************/
void MCP_CAN::clearBufferTransmitIfFlags(byte flags)
{
}


/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
byte MCP_CAN::checkError(void)
{
}






/*********************************************************************************************************
** Function name:           getCanId
** Descriptions:            when receive something, you can get the can id!!
*********************************************************************************************************/
unsigned long MCP_CAN::getCanId(void)
{
    return can_id;
}

/*********************************************************************************************************
** Function name:           isRemoteRequest
** Descriptions:            when receive something, you can check if it was a request
*********************************************************************************************************/
byte MCP_CAN::isRemoteRequest(void)
{
#undef rtr;
    return rtr;
#define rtr flags.remote
}

/*********************************************************************************************************
** Function name:           isExtendedFrame
** Descriptions:            did we just receive standard 11bit frame or extended 29bit? 0 = std, 1 = ext
*********************************************************************************************************/
byte MCP_CAN::isExtendedFrame(void)
{
    return ext_flg;
}



/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message by using buffer read as free from CANINTF status
**                          Status has to be read with readRxTxStatus and filtered with checkClearTxStatus
*********************************************************************************************************/
byte MCP_CAN::sendMsgBuf(byte status, unsigned long id, byte extended, byte rtrBit, byte len, volatile const byte *buf)
{
    CAN_message_t frame;
    ( rtrBit ) ? frame.rtr = 1 : frame.rtr = 0;
    ( extended ) ? frame.ext = 1 : frame.ext = 0;
    frame.id = id;
    frame.len = len;
    for ( uint8_t i = 0; i < 8; i++ ) frame.buf[i] = buf[i];
    Can0.write(frame);
    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           trySendMsgBuf
** Descriptions:            Try to send message. There is no delays for waiting free buffer.
*********************************************************************************************************/
byte MCP_CAN::trySendMsgBuf(unsigned long id, byte extended, byte rtrBit, byte len, const byte *buf, byte iTxBuf)
{
    CAN_message_t frame;
    ( rtrBit ) ? frame.rtr = 1 : frame.rtr = 0;
    ( extended ) ? frame.ext = 1 : frame.ext = 0;
    frame.id = id;
    frame.len = len;
    for ( uint8_t i = 0; i < 8; i++ ) frame.buf[i] = buf[i];
    Can0.write(frame);
    return CAN_OK;
}




/*********************************************************************************************************
** Function name:           mcp2515_start_transmit
** Descriptions:            Start message transmit on mcp2515
*********************************************************************************************************/
void MCP_CAN::mcp2515_start_transmit(const byte mcp_addr)              // start transmit
{
}

/*********************************************************************************************************
** Function name:           mcp2515_isTXBufFree
** Descriptions:            Test is tx buffer free for transmitting
*********************************************************************************************************/
byte MCP_CAN::mcp2515_isTXBufFree(byte *txbuf_n, byte iBuf)           /* get Next free txbuf          */
{
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Descriptions:            finds next free tx buffer for sending. Return MCP_ALLTXBUSY, if there is none.
*********************************************************************************************************/
byte MCP_CAN::mcp2515_getNextFreeTXBuf(byte *txbuf_n)                 // get Next free txbuf
{
}

/*********************************************************************************************************
** Function name:           enableTxInterrupt
** Descriptions:            enable interrupt for all tx buffers
*********************************************************************************************************/
void MCP_CAN::enableTxInterrupt(bool enable)
{
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
byte MCP_CAN::init_Mask(byte num, byte extended, unsigned long ulData)
{
    filter_masks[constrain(num,0,1)] = ulData;
    mask_ext_bit[constrain(num,0,1)] = extended;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************************************************************************************************/
byte MCP_CAN::init_Filt(byte num, byte extended, unsigned long ulData)
{
    filter_filters[constrain(num,0,5)] = ulData;
    filter_ext_bit[constrain(num,0,5)] = extended;
}


/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_read_id(const byte mcp_addr, byte* extended, unsigned long* id)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
**                          Note! There is no check for right address!
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_canMsg(const byte buffer_sidh_addr, unsigned long id, byte extended, byte rtrBit, byte len, volatile const byte *buf)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void MCP_CAN::mcp2515_write_id(const byte mcp_addr, const byte extended, const unsigned long id)
{
}



/*********************************************************************************************************
** Function name:           mcp2515_id_to_buf
** Descriptions:            configure tbufdata[4] from id and ext
*********************************************************************************************************/
void mcp2515_id_to_buf(const byte extended, const unsigned long id, byte *tbufdata)
{
}


/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
byte MCP_CAN::mcp2515_init(const byte canSpeed, const byte clock)
{
}




/*********************************************************************************************************
** Function name:           txCtrlReg
** Descriptions:            return tx ctrl reg according to tx buffer index.
**                          According to my tests this is faster and saves memory compared using vector
*********************************************************************************************************/
byte txCtrlReg(byte i) {
}

/*********************************************************************************************************
** Function name:           statusToBuffer
** Descriptions:            converts CANINTF status to tx buffer index
*********************************************************************************************************/
byte statusToTxBuffer(byte status)
{
}

/*********************************************************************************************************
** Function name:           statusToBuffer
** Descriptions:            converts CANINTF status to tx buffer sidh
*********************************************************************************************************/
byte statusToTxSidh(byte status)
{
}

/*********************************************************************************************************
** Function name:           txSidhToTxLoad
** Descriptions:            return tx load command according to tx buffer sidh register
*********************************************************************************************************/
byte txSidhToRTS(byte sidh) {
}

/*********************************************************************************************************
** Function name:           txSidhToTxLoad
** Descriptions:            return tx load command according to tx buffer sidh register
*********************************************************************************************************/
byte txSidhToTxLoad(byte sidh) {
}

/*********************************************************************************************************
** Function name:           txIfFlag
** Descriptions:            return tx interrupt flag
*********************************************************************************************************/
byte txIfFlag(byte i) {
}

/*********************************************************************************************************
** Function name:           txStatusPendingFlag
** Descriptions:            return buffer tx pending flag on status
*********************************************************************************************************/
byte txStatusPendingFlag(byte i) {
}

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void MCP_CAN::mcp2515_reset(void)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
byte MCP_CAN::mcp2515_readRegister(const byte address)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_readRegisterS(const byte address, byte values[], const byte n)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegister(const byte address, const byte value)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void MCP_CAN::mcp2515_setRegisterS(const byte address, const byte values[], const byte n)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void MCP_CAN::mcp2515_modifyRegister(const byte address, const byte mask, const byte data)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
byte MCP_CAN::mcp2515_readStatus(void)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
byte MCP_CAN::mcp2515_setCANCTRL_Mode(const byte newmode)
{
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            set baudrate
*********************************************************************************************************/
byte MCP_CAN::mcp2515_configRate(const byte canSpeed, const byte clock)
{
    Can0.setBaudRate(canSpeed);
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void MCP_CAN::mcp2515_initCANBuffers(void)
{
}



/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
