#ifndef NETWTWI_H
#define NETWTWI_H

#include "NetwBase.h"

// #define DEBUG

/*
 *
 * */

// #define SUPPORT_ON_REQUEST  // sync twi we use async

#define TWI_BUFFER_LENGTH   	20
#define TWI_TIMEOUT 			8000L			// restart after 21 seconds inactivity

#define TWI_READY 0
#define TWI_MRX   1

#define TWI_MTX   2

#define TWI_SRX   3
#define TWI_STX   4

class NetwTWI: public NetwBase
{
private:

	#ifdef SUPPORT_ON_REQUEST
		void (*user_onRequest)(void);
		void onRequestService(void);
	#endif

public:
	volatile uint8_t 	tw_slarw;
	volatile uint8_t 	tw_state;

    volatile byte slaveAdr = 0x00;
    volatile byte twUploadNode = 0x09;  // default for upload

	// not jet used
//	volatile uint8_t 	tw_sendStop;			// should the transaction end with a stop
//	volatile uint8_t 	tw_inRepStart;			// in the middle of a repeated start

	// volatile unsigned long timeOut = TWI_RESTART;
	volatile uint8_t 	tw_stopIssued;
	volatile uint8_t 	tw_error;

	volatile uint8_t tw_masterBuffer[TWI_BUFFER_LENGTH];
	volatile uint8_t tw_masterBufferIndex;
	volatile uint8_t tw_masterBufferLength;

//	CxData tw_txBuffer; // moved to NetwBase
////	uint8_t tw_txBuffer[TWI_BUFFER_LENGTH];
//	volatile uint8_t tw_txBufferIndex;
//	volatile uint8_t tw_txBufferLength;

	volatile uint8_t tw_rxBuffer[TWI_BUFFER_LENGTH];
	volatile uint8_t tw_rxBufferIndex=0;
	// volatile unsigned long tw_rxTimeOut=0;

	volatile unsigned int err40Count=0;
	volatile unsigned int err41Count=0;
	volatile unsigned int err42Count=0;
	volatile unsigned int err43Count=0;
	volatile unsigned int err44Count=0;
	volatile unsigned int err45Count=0;
	volatile unsigned int err46Count=0;
	volatile unsigned int err47Count=0;
	volatile unsigned int err48Count=0;
	volatile unsigned int err49Count=0;
	volatile unsigned int err50Count=0;
	volatile unsigned int err51Count=0;
	volatile unsigned int err52Count=0;
	volatile unsigned int err53Count=0;
	volatile unsigned int err54Count=0;
	volatile unsigned int err55Count=0;

	NetwTWI(){}
	virtual ~NetwTWI(){}  // suppress warning

//	static void (*twi_onSlaveTransmit)(void);
//    void onReceive( void (*)(int) );

    void onRequest( void (*)(void) );
	void begin();
	void tw_restart(void);
	void tw_releaseBus(void);
	void tw_disable(void);
	void tw_setAddress(uint8_t address);
	void tw_int(void);
	void tw_reply(uint8_t ack);
	void tw_stop(void);
	void tw_stopMTX(void);

	void txCommit(void);
	void rxCommit(void);

    int upload(int id);
	void localCmd(int cmd, long val);	
	void loop(void);

    void trace(char* id);

    bool isReady(void);
	bool isBusy(void);

	int write(RxData *rxData);

	void pullUpsOff()
	{
	  digitalWrite(SDA, 0);
	  digitalWrite(SCL, 0);
	}
	void pullUpsOn()
	{
	  digitalWrite(SDA, 1);
	  digitalWrite(SCL, 1);
	}

	void openLines(int digitalPort);
};

#endif
