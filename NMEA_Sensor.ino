//-------------------------------------------
// NMEA_Sensor.cpp
//-------------------------------------------
// Note that the monitor needs DEBUG_RXANY compile flag.

#include <myDebug.h>

#define HOW_BUS_MPC2515		0		// use github/autowp/arduino-mcp2515 library
#define HOW_BUS_CANBUS		1		// use github/_ttlappalainen/CAN_BUS_Shield mpc2515 library
#define HOW_BUS_NMEA2000	2		// use github/_ttlappalainen/CAN_BUS_Shield + ttlappalainen/NMEA2000 libraries

#define HOW_CAN_BUS			HOW_BUS_NMEA2000

#define MSG_SEND_TIME		1000
	// once per second; have had it work at 50ms (20 per second)
#define CAN_CS_PIN			5


#if HOW_CAN_BUS == HOW_BUS_MPC2515

	#include <mcp2515.h>
	#include <SPI.h>

	#define WITH_RETRIES				0		// whether to use ACK and do retries or not
	#define dbg_ack						1		// only used if WITH_RETRIES
	#define CANBUS_RETRIES 				3		// only used if WITH_RETRIES
	#define CAN_ACK_ID 					0x037	// only used if WITH_RETRIES
	#define CANBUS_ACK_TIMEOUT			1000	// only used if WITH_RETRIES

	#define MY_TEMPERATURE_CANID		0x036

	struct MCP2515 mcp2515(CAN_CS_PIN);

#elif HOW_CAN_BUS == HOW_BUS_CANBUS

	#include <mcp_can.h>
	#define SEND_NODE_ID				237
	MCP_CAN	canbus(CAN_CS_PIN);

#elif HOW_CAN_BUS == HOW_BUS_NMEA2000

	#include <NMEA2000_mcp.h>
	#include <N2kMessages.h>

	// forked and added API to pass the CAN_500KBPS baudrate

	tNMEA2000_mcp nmea2000(CAN_CS_PIN,MCP_8MHz,CAN_500KBPS);
	
	const unsigned long TransmitMessages[] = {130316L,0};

#endif



//------------------------
// setup
//------------------------

void setup()
{
	Serial.begin(921600);
	delay(2000);
	display(0,"NMEA_Sensor.ino setup(%d) started",HOW_CAN_BUS);
	Serial.println("WTF");

	#if HOW_CAN_BUS == HOW_BUS_MPC2515

		SPI.begin();
		mcp2515.reset();
		mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
		mcp2515.setNormalMode();

	#elif HOW_CAN_BUS == HOW_BUS_CANBUS

		byte err = canbus.begin(CAN_500KBPS, MCP_8MHz);
		while (err != CAN_OK)
		{
			my_error("canbus.begin() err=%d",err);
			delay(500);
			err = canbus.begin(CAN_500KBPS, MCP_8MHz);
		}

	#elif HOW_CAN_BUS == HOW_BUS_NMEA2000

		#if 0
			nmea2000.SetN2kCANMsgBufSize(150);
			nmea2000.SetN2kCANSendFrameBufSize(150);
			nmea2000.SetN2kCANReceiveFrameBufSize(150);
		#endif

		#if 0
			nmea2000.SetProductInformation(
				"00000001", 				// Manufacturer's Model serial code
				100, 						// Manufacturer's product code
				"Simple Temp Sensor",  		// Manufacturer's Model ID
				"1.0", 						// Manufacturer's Software version code
				"1.0" 						// Manufacturer's Model version
				);
			nmea2000.SetDeviceInformation(
				112233, // Unique number. Use e.g. Serial number.
				130, 	// Device function=Temperature. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
				75, 	// Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
				2040 	// Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
				);
		#endif

		// set its initial bus address to 22

		nmea2000.SetMode(tNMEA2000::N2km_SendOnly,	22);
			// N2km_NodeOnly
			// N2km_ListenAndNode
			// N2km_ListenAndSend
			// N2km_ListenOnly
			// N2km_SendOnly

		#if 1
			nmea2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
		#else
			nmea2000.SetForwardStream(&Serial);
			nmea2000.SetForwardType(tNMEA2000::fwdt_Text);
				// Show in clear text.
			nmea2000.SetForwardOwnMessages(true);
		#endif

		#if 0
			// I could not get this to eliminate need for DEBUG_RXANY
			// compiile flag in the Monitor

			nmea2000.ExtendTransmitMessages(TransmitMessages);
		#endif

		if (!nmea2000.Open())
			my_error("nmea2000.Open() failed",0);

	#endif	// HOW_CAN_BUS == HOW_BUS_NMEA2000

	display(0,"NMEA_Sensor.ino setup() finished",0);
}



//--------------------------------------------
// loop()
//--------------------------------------------

void loop()
{
	static float dir = 1;
	static uint32_t counter;
	static float temperatureC = 20;
	static uint32_t last_send_time;

	uint32_t now = millis();
	if (now - last_send_time > MSG_SEND_TIME)
	{
		last_send_time = now;

		temperatureC += dir;
		if (temperatureC > 100)
			dir = -1;
		else if (temperatureC < -100)
			dir = 1;

		display(0,"Sending(%d): %0.3fC",++counter,temperatureC);

		#if HOW_CAN_BUS == HOW_BUS_NMEA2000

			// display(0,"PGN(%d)=0x%08x",130316L,130316L);
			// PGN(130316L)	=	0x0001fd0c		TemperatureExt
			// PGN(60928L)	=	0x0000ee00		Address Claim
			// PGN(126993L)	=	0x0001f011		Heartbeet

			// SetN2kTemperatureExt is an alias for SetN2kPGN130316
			// Note tht degrees are Kelvin

			tN2kMsg N2kMsg; 	// it's a class, not a structure!
			double tempDouble = temperatureC;

			SetN2kPGN130316(
				N2kMsg,
				255,								// unsigned char SID; 255 indicates "unused"
				93,									// unsigned char TempInstance "should be unique per device-PGN"
				N2kts_RefridgerationTemperature,	// tN2kTempSource enumerated type
				CToKelvin(tempDouble),
				N2kDoubleNA							// double SetTemperature
			);

			// SIDS have the semantic meaning of tying a number of messages together to
			// 		one point in time.  If used, they should start at 0, and recyle after 252
			// 		253 and 254 are reserved.

			nmea2000.SendMsg(N2kMsg);

		#elif HOW_CAN_BUS == HOW_BUS_CANBUS

			// apparently this API has no notion of message ID's or ACKs,
			// and just sends whatever you give it.  So, here we just send
			// the temperature as 4 bytes with no further identification
			// besides the SEND_NODE_ID

			uint8_t *ptr = (uint8_t *) &temperatureC;
			display(0,"    sending ID(%d)=0x%08x",SEND_NODE_ID,SEND_NODE_ID);
			display_bytes(0,"    sendMsgBuf()",ptr,4);
			canbus.sendMsgBuf(SEND_NODE_ID, 0, 4, ptr);
				// id, 0=standard frame, 4=data_len

		#elif HOW_CAN_BUS == HOW_BUS_MPC2515

			uint8_t *ptr = (uint8_t *) &temperatureC;

			struct can_frame canMsg;
			memset(&canMsg,0,sizeof(canMsg));
				// can_id is 4 bytes
				// can_dlc is 1 byte (takes 4 bytes)
				// data bytes are aligned to 8 byte boundries
				// so this packet is 12 bytes in length

			canMsg.can_id  = MY_TEMPERATURE_CANID;  // CAN ID
			canMsg.can_dlc = 4;      // Data length code (number of bytes)
			canMsg.data[0] = *ptr++;
			canMsg.data[1] = *ptr++;
			canMsg.data[2] = *ptr++;
			canMsg.data[3] = *ptr++;

			display_bytes(0,"canMsg",(uint8_t *)&canMsg,sizeof(canMsg));

			#if !WITH_RETRIES

				MCP2515::ERROR err = mcp2515.sendMessage(&canMsg);
					// ERROR_OK        = 0,
					// ERROR_FAIL      = 1,
					// ERROR_ALLTXBUSY = 2,
					// ERROR_FAILINIT  = 3,
					// ERROR_FAILTX    = 4,
					// ERROR_NOMSG     = 5
				if (err != MCP2515::ERROR_OK)
					my_error("MCP2515::ERROR(%d)",err);

			#else

				bool messageSent = false;
				int retries = 0;

				while (!messageSent && retries < CANBUS_RETRIES)
				{
					if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK)
					{
						// Wait for acknowledgment
						unsigned long startTime = millis();
						bool ackReceived = false;

						while (millis() - startTime < CANBUS_ACK_TIMEOUT)
						{
							// Wait up to 500ms for an ACK
							if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
							{
								if (canMsg.can_id == CAN_ACK_ID)
								{
									ackReceived = true;
									break;
								}
								// delay(20);
							}
						}

						if (ackReceived)
						{
							display(dbg_ack,"ACK received",0);
							messageSent = true;
						}
						else
						{
							retries++;
							warning(0,"ACK not received, retrying(%d)...",retries);
							// delay(100);
						}
					}
					else
					{
						retries++;
						warning(0,"Error sending message, retrying(%d)...",retries);
						// delay(100);
					}
				}

				if (!messageSent)
				{
					my_error("Failed to send message(%d) after %d retries",counter,CANBUS_RETRIES);
				}
			#endif 	// WITH_RETRIES

		#endif	// HOW_CAN_BUS == HOW_BUS_MPC2515


	}	// time to send the message

	#if HOW_CAN_BUS == HOW_BUS_NMEA2000
		nmea2000.ParseMessages(); // Keep parsing messages
	#endif

	#if 0
		delay(2);
	#endif
	
}	// loop()
