#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <array>

using namespace std;


namespace comm_packet
{
const unsigned char HEADER  = 0xaa;
const unsigned char  FOOTER  = 0x55;
const unsigned char  RX_ID   = 0x20;
const unsigned char  TX_ID   = 0x10;

// size must be multiply of 4 for a memory alignement.
// EtherCAT -> PC
struct RxPacket{

	unsigned char header;
	unsigned char type;
	unsigned char counter;
	unsigned char length;

	int16_t       wheelVelFL;
	int16_t       wheelVelFR;
	int16_t       wheelVelRL;
	int16_t       wheelVelRR;
	int16_t		  wheelAcc;
	int16_t		  steerVelFL;
	int16_t		  steerVelFR;
	int16_t		  steerVelRL;
	int16_t		  steerVelRR;
	int16_t		  steerPosFL;
	int16_t		  steerPosFR;
	int16_t		  steerPosRL;
	int16_t		  steerPosRR;
	int16_t		  steerAcc;
	int16_t		  liftVelFL;
	int16_t		  liftVelFR;
	int16_t		  liftVelRL;
	int16_t		  liftVelRR;
	int16_t		  liftPosFL;
	int16_t		  liftPosFR;
	int16_t		  liftPosRL;
	int16_t		  liftPosRR;
	int16_t		  liftTrqFL;
	int16_t		  liftTrqFR;
	int16_t		  liftTrqRL;
	int16_t		  liftTrqRR;
	int16_t		  liftAcc;

	unsigned char liftDistMin;
	unsigned char liftDistMax;

	unsigned char controlCommand;
	unsigned char battery1SOC; // mobile battery SOC
	unsigned char battery2SOC; // manipulator battery SOC
	unsigned char reserved0;
	unsigned char chksum; // 0xff & (Byte #0 ^ ~Byte #55)
	unsigned char footer;
};

// PC -> EtherCAT
struct TxPacket {

	unsigned char header;
	unsigned char type;
	unsigned char counter;
	unsigned char length;

	int8_t       wheelVelFL_L;
	int8_t       wheelVelFL_H;
	int8_t       wheelVelFR_L;
	int8_t       wheelVelFR_H;
	int8_t       wheelVelRL_L;
	int8_t       wheelVelRL_H;
	int8_t       wheelVelRR_L;
	int8_t       wheelVelRR_H;
	int8_t		  wheelAcc_L;
	int8_t		  wheelAcc_H;

	int8_t		  steerVelFL_L;
	int8_t		  steerVelFL_H;
	int8_t		  steerVelFR_L;
	int8_t		  steerVelFR_H;
	int8_t		  steerVelRL_L;
	int8_t		  steerVelRL_H;
	int8_t		  steerVelRR_L;
	int8_t		  steerVelRR_H;
	int8_t		  steerPosFL_L;
	int8_t		  steerPosFL_H;
	int8_t		  steerPosFR_L;
	int8_t		  steerPosFR_H;
	int8_t		  steerPosRL_L;
	int8_t		  steerPosRL_H;
	int8_t		  steerPosRR_L;
	int8_t		  steerPosRR_H;
	int8_t		  steerAcc_L;
	int8_t		  steerAcc_H;

	int8_t		  liftVelFL_L;
	int8_t		  liftVelFL_H;
	int8_t		  liftVelFR_L;
	int8_t		  liftVelFR_H;
	int8_t		  liftVelRL_L;
	int8_t		  liftVelRL_H;
	int8_t		  liftVelRR_L;
	int8_t		  liftVelRR_H;

	int8_t		  liftPosFL_L;
	int8_t		  liftPosFL_H;
	int8_t		  liftPosFR_L;
	int8_t		  liftPosFR_H;
	int8_t		  liftPosRL_L;
	int8_t		  liftPosRL_H;
	int8_t		  liftPosRR_L;
	int8_t		  liftPosRR_H;

	int8_t		  liftTrqFL_L;
	int8_t		  liftTrqFL_H;
	int8_t		  liftTrqFR_L;
	int8_t		  liftTrqFR_H;
	int8_t		  liftTrqRL_L;
	int8_t		  liftTrqRL_H;
	int8_t		  liftTrqRR_L;
	int8_t		  liftTrqRR_H;
	int8_t		  liftAcc_L;
	int8_t		  liftAcc_H;

	unsigned char liftDistMin;
	unsigned char liftDistMax;

	unsigned char imuRoll;
	unsigned char imuPitch;
	unsigned char imuYaw;

	unsigned char controlCommand;
	unsigned char reserved0;

	unsigned char chksum;
	unsigned char footer;
};

}

#endif