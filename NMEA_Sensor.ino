//-------------------------------------------
// NMEA_Sensor.cpp
//-------------------------------------------
// Note that the monitor needs DEBUG_RXANY compile flag.

#include <myDebug.h>
#include <SPI.h>

#define dbg_sensor			0

#define USE_HSPI			0
	// use ESP32 alternative HSPI for mcp2515 so that it
	// isn't mucked with by the st7789 display
	// only currently supported with HOW_BUS_NMEA2000
#define CAN_CS_PIN			5
#define MSG_SEND_TIME		2000
	// once per second; have had it work at 50ms (20 per second)


#if USE_HSPI
	SPIClass *hspi;
		// MOSI=13
		// MISO=12
		// SCLK=14
		// default CS = 15
#endif



//-----------------------------------
// NMEA2000 Stuff
//-----------------------------------

#include <NMEA2000_mcp.h>
#include <N2kMessages.h>

#define BROADCAST_NMEA200_INFO	0

// forked and added API to pass the CAN_500KBPS baudrate

tNMEA2000_mcp nmea2000(CAN_CS_PIN,MCP_8MHz);	// ,CAN_500KBPS);

#define PGN_REQUEST					59904L
#define PGN_ADDRESS_CLAIM			60928L
#define PGN_PGN_LIST				126464L
#define PGN_HEARTBEAT				126993L
#define PGN_PRODUCT_INFO			126996L
#define PGN_DEVICE_CONFIG			126998L
#define PGN_TEMPERATURE    			130316L

const unsigned long AllMessages[] = {
	PGN_REQUEST,
	PGN_ADDRESS_CLAIM,
	PGN_PGN_LIST,
	PGN_HEARTBEAT,
	PGN_PRODUCT_INFO,
	PGN_DEVICE_CONFIG,
	PGN_TEMPERATURE,
	0};


void onNMEA2000Message(const tN2kMsg &mzg)
{
	#define SHOW_DETAILS	1

	#if SHOW_DETAILS
		display(dbg_sensor,"onNMEA2000 message(%d) priority(%d) source(%d) dest(%d) len(%d)",
			mzg.PGN,
			mzg.Priority,
			mzg.Source,
			mzg.Destination,
			mzg.DataLen);
		// also timestamp (ms since start [max 49days]) of the NMEA2000 message
		// unsigned long MsgTime;
	#else
		display(dbg_sensor,"onNMEA2000 message(%d)",mzg.PGN);
	#endif
}


//------------------------
// setup
//------------------------

void setup()
{
	Serial.begin(921600);
	delay(2000);
	display(dbg_sensor,"NMEA_Sensor.ino setup() started",0);
	Serial.println("WTF");

	#if 0
		nmea2000.SetN2kCANMsgBufSize(150);
		nmea2000.SetN2kCANSendFrameBufSize(150);
		nmea2000.SetN2kCANReceiveFrameBufSize(150);
	#endif.

	#if 1

		nmea2000.SetProductInformation(
			"prh_model_110",            // Manufacturer's Model serial code
			110,                        // Manufacturer's uint8_t product code
			"ESP32 NMEA Sensor",        // Manufacturer's Model ID
			"prh_sw_110.0",             // Manufacturer's Software version code
			"prh_mv_110.0",             // Manufacturer's uint8_t Model version
			1,                          // LoadEquivalency uint8_t 3=150ma; Default=1. x * 50 mA
			2101,                       // N2kVersion Default=2101
			1,                          // CertificationLevel Default=1
			0                           // iDev (int) index of the device on \ref Devices
			);
		nmea2000.SetConfigurationInformation(
			"prhSystems",           	// ManufacturerInformation
			"SensorInstall1",       	// InstallationDescription1
			"SensorInstall2"       		// InstallationDescription2
			);
		nmea2000.SetDeviceInformation(
			1230110, // uint32_t Unique number. Use e.g. Serial number.
			130,     // uint8_t  Device function=Temperature.. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
			75, 	 // Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
			2046     // uint16_t choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
			);
	#endif

	// set its initial bus address to 22

	nmea2000.SetMode(tNMEA2000::N2km_ListenAndNode,	22);
		// N2km_NodeOnly
		// N2km_ListenAndNode
		// N2km_ListenAndSend
		// N2km_ListenOnly
		// N2km_SendOnly

	#if 0
		nmea2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
	#else
		nmea2000.SetForwardStream(&Serial);
		nmea2000.SetForwardType(tNMEA2000::fwdt_Text);
			// Show in clear text.
		nmea2000.SetForwardOwnMessages(true);
	#endif

	#if 1
		// I could not get this to eliminate need for DEBUG_RXANY
		// compiile flag in the Monitor
		nmea2000.ExtendReceiveMessages(AllMessages);
		nmea2000.ExtendTransmitMessages(AllMessages);
	#endif

	#if	1
		nmea2000.SetMsgHandler(onNMEA2000Message);
	#endif

	#if USE_HSPI
		hspi = new SPIClass(HSPI);
		nmea2000.SetSPI(hspi);
	#endif

	if (!nmea2000.Open())
		my_error("nmea2000.Open() failed",0);

	display(dbg_sensor,"NMEA_Sensor.ino setup() finished",0);
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

	#define NUM_INFOS	4
	static int info_sent = 0;

	uint32_t now = millis();
	if (now - last_send_time > MSG_SEND_TIME)
	{
		last_send_time = now;

		#if BROADCAST_NMEA200_INFO

			// at this time I have not figured out the actisense reader, and how to
			// get the whole system to work so that when it asks for device configuration(s)
			// and stuff, we send it stuff.  However, this code explicitly sends some info
			// at boot, and I have seen the results get to the reader!

			if (info_sent < NUM_INFOS)
			{
				nmea2000.ParseMessages(); // Keep parsing messages
				switch (info_sent)
				{
					case 0:
						nmea2000.SendProductInformation();
							// 255,	// unsigned char Destination,
							// 0,		// only device
							// false);	// bool UseTP);
						break;
					case 1:
						nmea2000.SendConfigurationInformation(255,0,false);
						break;
					case 2:
						nmea2000.SendTxPGNList(255,0,false);
						break;
					case 3:
						nmea2000.SendRxPGNList(255,0,false);	// empty right now for the sensor
						break;
				}

				info_sent++;
				nmea2000.ParseMessages(); // Keep parsing messages
				return;
			}
		#endif	// BROADCAST_NMEA200_INFO

		temperatureC += dir;
		if (temperatureC > 100)
			dir = -1;
		else if (temperatureC < -100)
			dir = 1;

		display(dbg_sensor,"Sending(%d): %0.3fC",++counter,temperatureC);

		// display(0,"PGN(%d)=0x%08x",130316L,130316L);
		// PGN(130316L)	=	0x0001fd0c		TemperatureExt
		// PGN(60928L)	=	0x0000ee00		Address Claim
		// PGN(126993L)	=	0x0001f011		Heartbeet

		// SetN2kTemperatureExt is an alias for SetN2kPGN130316
		// Note tht degrees are Kelvin

		tN2kMsg mzg; 	// it's a class, not a structure!
		double tempDouble = temperatureC;

		SetN2kPGN130316(
			mzg,
			255,								// unsigned char SID; 255 indicates "unused"
			93,									// unsigned char TempInstance "should be unique per device-PGN"
			N2kts_RefridgerationTemperature,	// tN2kTempSource enumerated type
			CToKelvin(tempDouble),
			N2kDoubleNA							// double SetTemperature
		);

		// SIDS have the semantic meaning of tying a number of messages together to
		// 		one point in time.  If used, they should start at 0, and recyle after 252
		// 		253 and 254 are reserved.

		nmea2000.SendMsg(mzg);

	}	// time to send the message

	nmea2000.ParseMessages(); // Keep parsing messages

	#if 0
		delay(2);
	#endif
	
}	// loop()
