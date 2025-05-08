//-------------------------------------------
// NMEA_Sensor.cpp
//-------------------------------------------
// Note that the MCP2515 monitor needs DEBUG_RXANY compile flag.
//
// Pins to	MCP2515 Canbus module	pin 1 == VBUS
//
//		module			ESP32
//
//		(7) INT			GPIO22			orange
//		(6) SCK			GPIO18 (SCK)	white
//		(5) (MO)SI		GPIO23 (MOSI)  	yellow
//		(4) (MI)SO		GPIO19 (MISO)	green
// 		(3) CS			GPIO5	(CS)	blue
// 		(2) GND			GND				black
//		(1) VCC 	 	VBUS			red

#include <myDebug.h>
#include <NMEA2000_mcp.h>
#include <N2kMessages.h>
#include <SPI.h>

#define dbg_sensor			0
#define dbg_msgs			0


// Program Options

#define SHOW_BUS_MESSAGES		1
#define BROADCAST_NMEA200_INFO	0
#define BROADCAST_INTERVAL		300


//-------------------------------------------
// NMEA2000_mcp configuration
//-------------------------------------------

#define CAN_CS_PIN			5
#define CAN_INT_PIN			22		// 0xff for 'none'

#define USE_HSPI			0
	// use ESP32 alternative HSPI for mcp2515 so that it
	// isn't mucked with by the st7789 display
	// only currently supported with HOW_BUS_NMEA2000

#if USE_HSPI
	SPIClass *hspi;
		// MOSI=13
		// MISO=12
		// SCLK=14
		// default CS = 15
#endif

tNMEA2000_mcp nmea2000(CAN_CS_PIN,MCP_8MHz,CAN_INT_PIN);


//----------------------------------------------
// PGNs known by this program
//----------------------------------------------

#define PGN_REQUEST					59904L
#define PGN_ADDRESS_CLAIM			60928L
#define PGN_PGN_LIST				126464L
#define PGN_HEARTBEAT				126993L
#define PGN_PRODUCT_INFO			126996L
#define PGN_DEVICE_CONFIG			126998L


#define PGN_HEADING					127250L
#define PGN_SPEED					128259L
#define PGN_DEPTH					128267L
#define PGN_POSITION				129025L
#define PGN_TEMPERATURE    			130316L

const unsigned long TransmitMessages[] = {
#if 0
	PGN_REQUEST,
	PGN_ADDRESS_CLAIM,
	PGN_PGN_LIST,
	PGN_HEARTBEAT,
	PGN_PRODUCT_INFO,
	PGN_DEVICE_CONFIG,
#endif
	PGN_HEADING,
	PGN_SPEED,
	PGN_DEPTH,
	PGN_POSITION,
	PGN_TEMPERATURE,
	0
};


//-----------------------------------
// Program Configuration
//-----------------------------------

uint32_t sensor_interval 	= 0;			// no message sent until i<cr> or iNNN
	// a lone 'i' sends the sensors
	// iNNN sets an auto send interval

// any of the following commands also SEND the instrument

static double heading 		= 180;			// set by hNNN or hr for random (0..359)  	default = random
static double speed 		= 0;			// set by sNN  or sr for random (0..20)		default = 0
static double depth 		= 17;			// set by dNNN or dr for random (10-100)	default = random
static double latitude 		= 9.323584;		// fixed at this time
static double longitude 	= -82.237200;	// fixed at this time
static double temperatureF 	= 80;			// set by tNN or tr for random (0..110)		default = random

static bool random_heading 	= true;
static bool random_speed 	= false;
static bool random_depth 	= true;
static bool random_temp 	= true;


//-----------------------------------------------
// implementation
//-----------------------------------------------

static String usageMessage()
{
	String rslt = "Usage:\r\n";
	rslt += "   ? = Show this help\r\n";
	rslt += "   h,hr,hNNN = send Heading\r\n";
	rslt += "   s,sr,sNN = send Speed\r\n";
	rslt += "   d,dr,dNN = send Depth\r\n";
	rslt += "   t,tr,tNNN = send Temperature\r\n";
	rslt += "   i,iNNN = set sensor_interval cur=";
	rslt += String(sensor_interval);
	rslt += "\r\n";
	return rslt;
}



static void onNMEA2000Message(const tN2kMsg &msg)
{
	display(dbg_msgs,"onNMEA2000 message(%d) priority(%d) source(%d) dest(%d) len(%d)",
		msg.PGN,
		msg.Priority,
		msg.Source,
		msg.Destination,
		msg.DataLen);

	// also timestamp (ms since start [max 49days]) of the NMEA2000 message
	// unsigned long MsgTime;

	if (msg.PGN == PGN_REQUEST)
	{
		unsigned long requested_pgn;
		if (ParseN2kPGN59904(msg, requested_pgn))
			warning(dbg_msgs,"    PGN_REQUEST(%d)",requested_pgn);
	}
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
	Serial.print(usageMessage().c_str());

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

		// for device class see: https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		// 	archived at NMEA_Monitor/docs/20120726 nmea 2000 class & function codes v 2.00-1.pdf.
		// for the registration/company id, I guess 2046 arbitrarily chose because its NOT in
		//	https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
		// 	archived at NMEA_Monitor/docs/20121020 nmea 2000 registration list.pdf

		nmea2000.SetDeviceInformation(
			#if INSTRUMENT_TYPE==1
				1230111, // uint32_t my arbitrary unique (serial) number
				170,     // uint8_t  Function(60,170)=Integrated Navigation
				60, 	 // uint16_t Class=Navigation.
			#else
				1230110, // uint32_t my arbitrary unique (serial) number
				130,     // uint8_t  Function(75,130)=Temperature
				75, 	 // uint16_t Class(75)=Sensor Communication Interface
			#endif
			2046     // uint16_t Registration/Company) ID
					 // 2046 does not exist
			);
	#endif

	// set its initial bus address to 22

	nmea2000.SetMode(tNMEA2000::N2km_ListenAndNode,	22);
		// N2km_NodeOnly
		// N2km_ListenAndNode
		// N2km_ListenAndSend
		// N2km_ListenOnly
		// N2km_SendOnly

	#if SHOW_BUS_MESSAGES
		nmea2000.SetForwardStream(&Serial);
		nmea2000.SetForwardType(tNMEA2000::fwdt_Text);
			// Show in clear text.
		nmea2000.SetForwardOwnMessages(false);
	#else
		nmea2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
	#endif


	#if 1
		// I could not get this to eliminate need for DEBUG_RXANY
		// compiile flag in the Monitor
		//
		// I now believe that I should only send the messages
		// the sensor transmits here ...

		nmea2000.ExtendTransmitMessages(TransmitMessages);
			// nmea2000.ExtendReceiveMessages(AllMessages);
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
// sendSensor()
//--------------------------------------------

static void sendSensor(uint32_t PGN)
{
	static uint32_t counter;

	// note that these blocks each check the PGN
	// and then send that kind of PGN even though
	// the library does not have a polymorphic API

	tN2kMsg msg; 	// it's a class, not a structure!

	if (PGN == PGN_POSITION)
	{
		display(dbg_sensor,"Send(%d) lat(%0.6f) lon(%0.6f)",++counter,latitude,longitude);
		SetN2kPGN129025(msg, latitude, longitude);
			// PGN_POSITION - lat lon in double degrees
	}
	else if (PGN == PGN_HEADING)
	{
		if (random_heading)
		{
			static double inc = 5;
			heading += inc;
			if (heading >= 255) inc = -5;
			if (heading <= 5) inc = 5;
		}
		display(dbg_sensor,"Send(%d) heading: %0.3f",++counter,heading);
		SetN2kPGN127250(msg, 255, DegToRad(heading), 0.0 /*Deviation*/, 0.0 /*Variation*/, N2khr_true /* tN2kHeadingReference(0) */);
			// PGN_HEADING - heading is in radians
	}
	else if (PGN == PGN_SPEED)
	{
		if (random_speed)
		{
			static double inc = 0.2;
			speed += inc;
			if (speed >= 8) inc = -0.2;
			if (speed <= 2) inc = 0.2;
		}
		display(dbg_sensor,"Send(%d) speed: %0.3f",++counter,speed);
		SetN2kPGN128259(msg, 255, KnotsToms(speed));
			// PGN_SPEED - speed is meters/sec
	}
	else if (PGN == PGN_DEPTH)
	{
		if (random_depth)
		{
			static double inc = 1;
			depth += inc;
			if (depth > 99) inc = -1;
			if (depth < 11)  inc = 1;
		}
		display(dbg_sensor,"Send(%d) depth: %0.3f",++counter,depth);
		SetN2kPGN128267(msg, 255, depth, 2.0);
			// PGN_DEPTH - depth is in meters
			// with a 2.0 meter offset of keel
	}
	else if (PGN == PGN_TEMPERATURE)
	{
		// Note tht degrees are Kelvin

		if (random_temp)
		{
			static double dir = 1;
			temperatureF += dir;
			if (temperatureF > 109)
				dir = -1;
			else if (temperatureF < 1)
				dir = 1;
		}
		display(dbg_sensor,"Send(%d) temoerature: %0.3fF",++counter,temperatureF);
		tN2kTempSource temp_kind = N2kts_MainCabinTemperature;	// 4
			// tN2kTempSource temp_kind = N2kts_RefridgerationTemperature;	// 7

		SetN2kPGN130316(						// PGN_TEMPERATURE
			msg,
			255,								// unsigned char SID; 255 indicates "unused"
			93,									// unsigned char TempInstance "should be unique per device-PGN"
			temp_kind,
			FToKelvin(temperatureF),
			N2kDoubleNA							// double SetTemperature
		);
	}

	// SIDS have the semantic meaning of tying a number of messages together to
	// 		one point in time.  If used, they should start at 0, and recyle after 252
	// 		253 and 254 are reserved.

	nmea2000.SendMsg(msg);

}	// sendSensor()



static void sendSensors()
{
	sendSensor(PGN_HEADING);
	sendSensor(PGN_SPEED);
	sendSensor(PGN_DEPTH);
	sendSensor(PGN_TEMPERATURE);
}



//-----------------------------
// handleSerial()
//-----------------------------


static void handleCommand(uint32_t PGN,const char *name,char *linebuf,bool *random, double *value)
{
	if (linebuf[0] == 'r')
	{
		*random = true;
		warning(0,"Setting random %s",name);
	}
	else if (linebuf[0] != 0)
	{
		*random = false;
		*value = (double) atol(linebuf);
		warning(0,"Setting %s=%0.1f",name,*value);
	}
	sendSensor(PGN);
}



static void handleSerial()
{
	#define MAXLINE 80
	static bool in_line;

	static uint8_t line_ptr;
	static char linebuf[MAXLINE+1];
	static uint8_t line_command;

	if (Serial.available())
	{
		uint8_t byte = Serial.read();
		if (in_line)
		{
			if (byte == 0x0A || line_ptr >= MAXLINE)
			{
				linebuf[line_ptr++] = 0;

				if (line_command == 'i')
				{
					if (linebuf[0] == 0)	// i by itself
					{
						sensor_interval = 0;
						sendSensors();
					}
					else
					{
						sensor_interval = atol(linebuf);
						warning(0,"Setting Sensor Interval=%d",sensor_interval);
					}
				}
				else if (line_command == 'h')
					handleCommand(PGN_HEADING,"heading",linebuf,&random_heading,&heading);
				else if (line_command == 's')
					handleCommand(PGN_SPEED,"speed",linebuf,&random_speed,&speed);
				else if (line_command == 'd')
					handleCommand(PGN_DEPTH,"depth",linebuf,&random_depth,&depth);
				else if (line_command == 't')
					handleCommand(PGN_TEMPERATURE,"temperature",linebuf,&random_temp,&temperatureF);

				line_command = 0;
				line_ptr = 0;
				in_line = 0;
			}
			else if (byte != 0x0D)
			{
				linebuf[line_ptr++] = (char) byte;
			}
		}
		else if (byte == '?')
			Serial.print(usageMessage().c_str());
		else if (
			byte == 'i' ||
			byte == 'h' ||
			byte == 's' ||
			byte == 'd' ||
			byte == 't')
		{
			in_line = 1;
			line_ptr = 0;
			line_command = byte;
		}
		else if (byte == 'p')
		{
			sendSensor(PGN_POSITION);
		}
		else
		{
			warning(0,"unhandled serial character(%c) 0x%02x",byte>32?byte:' ',byte);
		}
	}
}



//--------------------------------------------
// loop()
//--------------------------------------------

void loop()
{
	uint32_t now = millis();

	#if BROADCAST_NMEA200_INFO

		// at this time I have not figured out the actisense reader, and how to
		// get the whole system to work so that when it asks for device configuration(s)
		// and stuff, we send it stuff.  However, this code explicitly sends some info
		// at boot, and I have seen the results get to the reader.

		#define NUM_INFOS	4
		static int info_sent = 0;
		static uint32_t broadcast_time;

		if (info_sent < NUM_INFOS && now - broadcast_time > BROADCAST_INTERVAL)
		{
			broadcast_time = now;
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


	//------------------------------------
	// send repeating sensor data
	//-----------------------------------

	static uint32_t sensor_time;
	if (sensor_interval && now - sensor_time > sensor_interval)
	{
		sensor_time = now;
		sendSensors();
	}

	//----------------------------
	// general loop handling
	//----------------------------

	nmea2000.ParseMessages(); // Keep parsing messages

	handleSerial();
	
	#if 0	// bad idea in time critical loop
		delay(2);
	#endif
	
}	// loop()
