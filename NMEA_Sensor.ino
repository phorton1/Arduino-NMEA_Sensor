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


//--------------------------------------------------------------------
// Raymarine E80 Info
//--------------------------------------------------------------------
// 2025-05-16 More detailed experiments with NMEA2000 <--> E80
//		Today I can see that the E80 is receiving *something* from the NMEA200 bus,
//			as the count of "frames" on the "Diagnostics-SeaTalkNG" increases when I
//			put stuff on the bus via either my  "monitor" or the "sensor".
//		However, I have yet to get anything meaningful to show up on the E80.
//			- I had to do a "Settings Reset" from the "System Setup Menu" on the E80
//			  I think, to get it to forget that I have also talked to it with the
//			  NMEA0183 simulator, which is now out of the circuit.
//          - Even then I have never seen anything that looks like the actual bytes
//			  I am sending show up in the E80's "buffer", which always just shows
//			  00000000 0B 00 0C 00 01 78 6F 01
//			- The E80 appears to need the NMEA2000 bus to have 12V, BUT if it has
//			  12V when the E80 is powered up, it gets 100K's of 'frames' during boot.
//			  Therefore, at this stage of testing, I boot the E80, and THEN hook the 12V
//			  upto the NMEA200 bus.
//		I *have* ascertained that the E80 and my breadboards are all working at 250Kbps,
//			   the standard NMEA2000 CANBUS speed.
//		With an oscilloscope, the combined signal with the E80 does not look particularly clean.
//			The "purepure" signal on my "bus" MCP2515 bus appears to be fairly clean with the
//			HIGH  going from 2.5 upto 3.5V as expected, and the low appears to from 2.5V down
//			to 1.5V as expected.
//
//			As soon as the E80 is connected, the signals appear to degrade, as if the
//			    MCP2515 doesn't have enough "oomph" to work correctly with the E80.
//				The LOW Signal apears slow to rise back to 2.5V, curving to the right,
//					as if the E80 is pulling it down.
//				The HIGH Signal also appears to be pulled down on the falling edges,
//					falling past 1.5V down to a sharp bottom peak at about 0.5V, and
//					then rising to 1.5 in a curv of about 10us.  I have a feeling
//					this *may* be being interpreted by the E80 as a risign edge, thus
//					causing it to receive garbage.
//			more rounded.  the HIGH signal appears to (perhaps) be being pulled DOWN
//			by the E80 more than the ST2515 module can pull it up.  As I understand it, the
//			"middle" of the signal should be about 2.5V, and the HIGH should pull it up by about
//			a volt to 3.5V or so, and the LOW should pull it down to 1.5V or so.
//
//		I decided to rework the breadboard so that I could experiment with pullups on
//			the data lines.
//
// JOY!!!
//
//		I have no idea what I changed to get it to start working.
//		I changed the breadboard so that I could experiment.
//			Instead of hooking the E80 up to one, the and oscilliscope up
//				to the other of the screw terminals of the ST2515's, I wired
//				it up so that they are "joined" in the middle of the breadboard,
//				so I could create a "drop" and hook the oscilliscope up more
//				easily, running short dupont male wires from the screw terminals
//				to the center of the breadboard.
//			I also pulled the 12V and ground from the power supply from the screw
//				in Pheonix connector to the E80 cable, and made them separate wires
//				to the breadboard.
//				There is one common ground, including both ESP32s, and the E80 cable.
//				The 12V positive IS hooked up to the E80 red wire,
//				but nowhere else on the breadboard.
//			I was still getting a funky looking waveform, so I decided to try to
//				"tame" the 12V by putting a 10K resistor between the power supply and the
//				12V on the E80 cable.  No joy, and I noticed the LOW signal was kind of
//				wonky.
//			After reworking the ST2515 connectors to fix the wonky LOW signal, I started
//				getting better looking waveforms.
//			I turned on the E80, and lo and behold, no 1000's of bad messages, but tood
//				messages AND IT STARTED SHOWING THE DEPTH!
//			In fact, the E80 even showed up in the device list and is regurgitating
//				the depth that I send from the sensor back out as its own messaes (source=160)!!
//
//				Raymarine E80
//				    Source: 160
//				    Manufacturer code:  1851
//				    Unique number:      108087
//				    Software version:   5.69
//				    Model version:      1.0
//				    Transmit PGNs :
//					59392, ]59904, 60928, 61184, 65288, 65311, 65361, 65362,
//					65364, 126208, 126464, 126720, 126992, 126996, 127250,
//					128259, 128267, 128275, 129025, 129026, 129029, 129033,
//					129044, 129283, 129284, 129291, 129540, 130306, 130310, 130577, 1308
//
//	Yippie yay oh kay aye!
//
//	Then it stopped working, and the E80 does not appear to be getting my messaes correctly.
//  I'm gonna try to reboot the E80.
//
//	After rebooting everything it started working correctly again.
//
//	I notice on the oscilliscope that the messaage coming from the E80 are noticably
//  larger (+/-1.25V instead of +/- 1V) and somewhat sharper.
//
// I wonder if the whole problem was using the stupid female dupont connectors
// on the ST2515 ?!?
//
// I am considering changing the "sensor" to "be" a NMEA0183->NMEA2000 converter,
//		factoring the "simulator" out of the Arduino NMEA1083 and Seatalk projects,
//		and adding it to this program.
//
// Other potential experiments include seeing if the E80 will forward Seatalk to the NMEA200
//		bus (maybe it needed a listener?!?), and same with NMEA0183.
//
//		It did not work the first time with the NMEA0183 simulator.
//			I wonder if you can only have one kind of NMEA network going
//				at a time?
//				2nd Try, start by just getting the NMEA0183 working again.
//
//		It DID work the first time with the Seatalk simulator, which I then
//			modified to NOT send the depth to see if the E80 would get it
//			from NMEA2000, which it did!

// Raymarine E80 Transmitted PGNs:
// 		59392 - ISO Acknowledgment
// 		59904 - ISO Request
// 		60928 - ISO Address Claim
// 		61184 - Proprietary Message
// 		65288 - Manufacturer Proprietary Data
// 		65311 - Manufacturer Proprietary Data
// 		65361 - Manufacturer Proprietary Data
// 		65362 - Manufacturer Proprietary Data
// 		65364 - Manufacturer Proprietary Data
// 		126208 - NMEA Command/Request/Acknowledge
// 		126464 - PGN List
// 		126720 - Proprietary Message
// 		126992 - System Time
// 		126996 - Product Information
// 		127250 - Vessel Heading
// 		128259 - Speed, Water Referenced
// 		128267 - Water Depth
// 		128275 - Distance Log
// 		129025 - Position, Rapid Update
// 		129026 - COG & SOG, Rapid Update
// 		129029 - GNSS Position Data
// 		129033 - Time & Date
// 		129044 - Datum
// 		129283 - Cross Track Error
// 		129284 - Navigation Data
// 		129291 - Navigation Route/WP Info
// 		129540 - GNSS Satellites in View
// 		130306 - Wind Data
// 		130310 - Environmental Parameters
// 		130577 - Direction Data
// 		1308 - Unknown PGN (Possibly incomplete)
//
// PGNS Relating to Engines and Generators
//
//		127488 - Engine Parameters, Rapid Update
//		127489 - Engine Parameters, Dynamic
//		127493 - Transmission Parameters, Dynamic
//		127496 - Engine Parameters, Static
//		127497 - Transmission Parameters, Static
//		127498 - Engine Trip Parameters
//		127503 - AC Generator Status
//		127504 - AC Generator Phase Parameters
//
// Engine PGNs in more detail
//
//		127488 - Engine Parameters, Rapid Update
//		  "EngineSpeed: 3200, BoostPressure: 1.2, TiltTrim: 45"
//		  EngineSpeed - RPM of the engine
//		    - Type: Integer, Units: RPM
//		  BoostPressure - Pressure in the intake manifold
//		    - Type: Float, Units: Bar
//		  TiltTrim - Angle of engine tilt
//		    - Type: Integer, Units: Percentage
//
//		127489 - Engine Parameters, Dynamic
//		  "EngineTemp: 85, AlternatorPotential: 13.8, FuelRate: 2.5, EngineHours: 1200"
//		  EngineTemp - Current engine temperature
//		    - Type: Float, Units: Celsius
//		  AlternatorPotential - Voltage output of alternator
//		    - Type: Float, Units: Volts
//		  FuelRate - Fuel consumption rate
//		    - Type: Float, Units: Liters per hour
//		  EngineHours - Cumulative engine runtime
//		    - Type: Integer, Units: Hours
//
//		127493 - Transmission Parameters, Dynamic
//		  "GearStatus: Forward, OilTemp: 75, Pressure: 2.1"
//		  GearStatus - Current gear selection (neutral, forward, reverse)
//		  OilTemp - Transmission oil temperature
//		    - Type: Float, Units: Celsius
//		  Pressure - Hydraulic pressure in transmission
//		    - Type: Float, Units: Bar
//
//		127496 - Engine Parameters, Static
//		  "RatedEngineSpeed: 6000"
//		  RatedEngineSpeed - Maximum rated RPM of the engine
//		    - Type: Integer, Units: RPM
//
//		127497 - Trip Parameters, Engine
//		  "FuelUsed: 50, EngineHours: 10"
//		  FuelUsed - Total fuel consumed during trip
//		    - Type: Float, Units: Liters
//		  EngineHours - Engine runtime for the trip
//		    - Type: Integer, Units: Hours
//
//		127498 - Engine Parameters, Static
//		  "RatedEngineSpeed: 6000"
//		  RatedEngineSpeed - Maximum rated RPM of the engine
//		    - Type: Integer, Units: RPM
//
//		127503 - AC Generator Status
//		  "GeneratorVoltage: 230, GeneratorCurrent: 10, GeneratorFrequency: 60"
//		  GeneratorVoltage - Output voltage of generator
//		    - Type: Float, Units: Volts
//		  GeneratorCurrent - Output current of generator
//		    - Type: Float, Units: Amperes
//		  GeneratorFrequency - Operating frequency of generator
//		    - Type: Float, Units: Hertz
//
//		127504 - AC Generator Phase Parameters
//		  "PhaseVoltage: 230, PhaseCurrent: 10, PowerFactor: 0.95"
//		  PhaseVoltage - Voltage per phase
//		    - Type: Float, Units: Volts
//		  PhaseCurrent - Current per phase
//		    - Type: Float, Units: Amperes
//		  PowerFactor - Efficiency of power conversion
//		    - Type: Float
//
// Raymarine "1308" Proprietary PGNs from Copilot
//		130842 - Autopilot Mode Control
//		  "Mode: Auto, HeadingSource: GPS, RudderLimit: 30"
//		  Mode - Current autopilot mode (Auto, Standby, Wind, Track)
//		  HeadingSource - Source of heading data (GPS, Compass)
//		  RudderLimit - Maximum rudder angle
//		    - Type: Integer, Units: Degrees
//
//		130843 - Autopilot Heading Command
//		  "TargetHeading: 120, TurnRate: 5"
//		  TargetHeading - Desired heading for autopilot
//		    - Type: Float, Units: Degrees
//		  TurnRate - Rate of turn applied by autopilot
//		    - Type: Float, Units: Degrees per second
//
//		130845 - Autopilot Status
//		  "CurrentHeading: 118, RudderAngle: 10, Mode: Auto"
//		  CurrentHeading - Vessel’s current heading
//		    - Type: Float, Units: Degrees
//		  RudderAngle - Current rudder position
//		    - Type: Float, Units: Degrees
//		  Mode - Active autopilot mode (Auto, Standby, Wind, Track)
//
//		130850 - Radar Target Tracking
//		  "TargetID: 5, Bearing: 45, Range: 2.5"
//		  TargetID - Identifier for tracked radar target
//		  Bearing - Direction to target
//		    - Type: Float, Units: Degrees
//		  Range - Distance to target
//		    - Type: Float, Units: Nautical Miles
//
//		130851 - Radar Range and Bearing
//		  "Range: 3.2, Bearing: 90"
//		  Range - Distance to detected object
//		    - Type: Float, Units: Nautical Miles
//		  Bearing - Direction to detected object
//		    - Type: Float, Units: Degrees
//
//		130860 - System Diagnostics
//		  "Voltage: 12.5, Temperature: 45, ErrorCode: 0"
//		  Voltage - System voltage level
//		    - Type: Float, Units: Volts
//		  Temperature - Internal system temperature
//		    - Type: Float, Units: Celsius
//		  ErrorCode - Diagnostic error code
//		    - Type: Integer
//


#include <myDebug.h>
#include <NMEA2000_mcp.h>
#include <N2kMessages.h>
#include <SPI.h>

#define dbg_sensor			0
#define dbg_in_msgs			0


// Program Options

#define BROADCAST_NMEA200_INFO	0
#define BROADCAST_INTERVAL		300


#define ALIVE_LED		2
#define ALIVE_OFF_TIME	980
#define ALIVE_ON_TIME	20


//-------------------------------------------
// NMEA2000_mcp configuration
//-------------------------------------------

#define CAN_CS_PIN			5
#define CAN_INT_PIN			0xff	// 22		// 0xff for 'none'
	// trying to turn off interrupt to solve WDT problem

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
#define PGN_ENGINE_RAPID			127488L
#define PGN_ENGINE_DYNAMIC			127489L
#define PGN_FLUID_LEVEL				127505L


const unsigned long TransmitMessages[] = {

	// these system PGNs may not be necessary here,
	// but it is more conformal to include them.
#if 1
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
	PGN_ENGINE_RAPID,
	PGN_ENGINE_DYNAMIC,
	PGN_FLUID_LEVEL,
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
static double rpms			= 1500;

bool show_bus = 0;
bool show_sends = 0;

static bool random_heading 	= true;
static bool random_speed 	= false;
static bool random_depth 	= true;
static bool random_temp 	= true;
static bool random_rpms		= true;


//-----------------------------------------------
// implementation
//-----------------------------------------------

static String usageMessage()
{
	String rslt = "NMEA Sensor Usage:\r\n";
	rslt += "   ? = Show this help\r\n";
	rslt += "   h,hr,hNNN = send Heading\r\n";
	rslt += "   s,sr,sNN = send Speed\r\n";
	rslt += "   d,dr,dNN = send Depth\r\n";
	rslt += "   t,tr,tNNN = send Temperature\r\n";
	rslt += "   r,rr,rNNN = send engine RPMs\r\n";
	rslt += "   i,iNNN = set sensor_interval cur=";
	rslt += String(sensor_interval);
	rslt += "\r\n";
	rslt += "   b = toggle show bus messages cur=";
	rslt += String(show_bus);
	rslt += "\r\n";
	rslt += "   m = show send messages cur=";
	rslt += String(show_sends);
	rslt += "\r\n";

	rslt += "   x! = reboot\n\n";
	return rslt;
}



static void onNMEA2000Message(const tN2kMsg &msg)
{
	if (show_bus)
	{
		#define MAX_DBG_BUF	512
		static int bus_num = 0;
		static char bus_buf[MAX_DBG_BUF+1];
		sprintf(bus_buf,"BUS(%d) : pri:%d PGN:%d src:%d dst:%d len:%d  data:",
			bus_num++,
			msg.Priority,
			msg.PGN,
			msg.Source,
			msg.Destination,
			msg.DataLen);
		int buf_len = strlen(bus_buf);
		for (int i=0; i<msg.DataLen && buf_len<MAX_DBG_BUF+3; i++)
		{
			sprintf(&bus_buf[buf_len],"%02x ",msg.Data[i]);
			buf_len += 3;
		}
		Serial.println(bus_buf);
		
		// display(dbg_in_msgs,"onNMEA2000 message(%d) priority(%d) source(%d) dest(%d) len(%d)",
		// 	msg.PGN,
		// 	msg.Priority,
		// 	msg.Source,
		// 	msg.Destination,
		// 	msg.DataLen);
	}

	// also timestamp (ms since start [max 49days]) of the NMEA2000 message
	// unsigned long MsgTime;

	if (msg.PGN == PGN_REQUEST)
	{
		unsigned long requested_pgn;
		if (ParseN2kPGN59904(msg, requested_pgn))
			warning(dbg_in_msgs,"    PGN_REQUEST(%d)",requested_pgn);
	}
}


//------------------------
// setup
//------------------------


// #include <soc/rtc_wdt.h>

void setup()
{
	#if ALIVE_LED
		pinMode(ALIVE_LED,OUTPUT);
		digitalWrite(ALIVE_LED,1);
	#endif

	// STILL getting spurious reboots from wdt
	// rtc_wdt_disable();

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

	#if 0	// SHOW_BUS_MESSAGES
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

	#if ALIVE_LED
		digitalWrite(ALIVE_LED,0);
	#endif
}



//--------------------------------------------
// sendSensor()
//--------------------------------------------

static void sendSensor(uint32_t PGN, unsigned char instance=0)
{
	static uint32_t counter;

	// note that these blocks each check the PGN
	// and then send that kind of PGN even though
	// the library does not have a polymorphic API

	tN2kMsg msg; 	// it's a class, not a structure!

	if (PGN == PGN_POSITION)
	{
		if (show_sends)
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
		if (show_sends)
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
		if (show_sends)
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
		if (show_sends)
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
		if (show_sends)
			display(dbg_sensor,"Send(%d) temperature: %0.3fF",++counter,temperatureF);
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
	else if (PGN == PGN_ENGINE_RAPID)
	{
		if (random_rpms)
		{
			rpms += random(-100,100);
			if (rpms > 3000) rpms = 3000;
			if (rpms < 0) rpms = 0;
		}
		if (show_sends)
			display(dbg_sensor,"Send(%d) rpms: %d",++counter,((int) rpms) );
		SetN2kPGN127488(
			msg,
			0					// EngineInstance
,			rpms,				// EngineSpeed
			N2kDoubleNA,		// EngineBoostPressure
			N2kDoubleNA);		// EngineTiltTrim
	}
	else if (PGN == PGN_ENGINE_DYNAMIC)
	{
		double oil_pressure = rpms == 0 ? 0 :
			50 + random(-30,30);
		double alt_voltage = rpms == 0 ? 0 :
			12.0 + (((float)random(-300,300)) / 100.0);
		double coolant_temp = rpms == 0 ? 0 :
			180 + random(-40,40);
		double fuel_rate = rpms == 0 ? 0 :
			1.5 + (((float) random(-100,100)) / 100.0);

		#define PSI_TO_PASCAL		6895.0
		#define GALLON_TO_LITRE		3.785

		if (show_sends)
			display(dbg_sensor,"Send(%d) temperature(%d) pressure(%d) voltage(%0.1f) rate(%0.1f)",
				++counter,
				((int) coolant_temp),
				((int) oil_pressure),
				alt_voltage,
				fuel_rate);

		static tN2kEngineDiscreteStatus1 status1;		// filled with zeros
		static tN2kEngineDiscreteStatus2 status2;		// filled with zeros

		SetN2kPGN127489(msg,
			0,									// EngineInstance
			oil_pressure * PSI_TO_PASCAL,		// EngineOilPress      in Pascal
			FToKelvin(coolant_temp),			// EngineOilTemp       in Kelvin
			FToKelvin(coolant_temp),			// EngineCoolantTemp   in Kelvin
			alt_voltage,						// AltenatorVoltage    in Voltage
			fuel_rate * GALLON_TO_LITRE,		// FuelRate            in litres/hour
			N2kDoubleNA,						// EngineHours         in seconds
			N2kDoubleNA,						// EngineCoolantPress  in Pascal
			N2kDoubleNA,						// EngineFuelPress     in Pascal
			0,									// EngineLoad          in %
			0,									// EngineTorque        in %
			status1,							// Status1             Engine Discrete Status 1
			status2);							// Status2             Engine Discrete Status 2
	}
	else if (PGN == PGN_FLUID_LEVEL)
	{
		double level = 50 + (float)random(-20,20);
		double capacity = 144 * GALLON_TO_LITRE;
		tN2kFluidType fluid_type = N2kft_Fuel;

		if (show_sends)
			display(dbg_sensor,"Send(%d) fuel_level(%d)=%d",
				++counter,instance,(int)level);

		SetN2kPGN127505(msg, instance, fluid_type, level, capacity);
	}
	
	// SIDS have the semantic meaning of tying a number of messages together to
	// 		one point in time.  If used, they should start at 0, and recyle after 252
	// 		253 and 254 are reserved.

	nmea2000.SendMsg(msg);

}	// sendSensor()



static void sendSensors()
{
	static int sensor_num = 0;

	#define MAX_SEND 	7

	if (!show_sends)
	{
		static int send_num = 0;
		if (send_num++ % (5 * MAX_SEND) == 0)
			display(0,"sent (%d) sensor messages",send_num);
	}
	
	switch (sensor_num)
	{
		// case 0:	sendSensor(PGN_HEADING);      break;
		// case 1:	sendSensor(PGN_SPEED);        break;
		case 1:	sendSensor(PGN_DEPTH);            break;
		case 2:	sendSensor(PGN_TEMPERATURE);      break;
		case 3:	sendSensor(PGN_ENGINE_RAPID);     break;
		case 4:	sendSensor(PGN_ENGINE_DYNAMIC);   break;
		case 5: sendSensor(PGN_FLUID_LEVEL,0);	  break;	// port fuel tank
		case 6: sendSensor(PGN_FLUID_LEVEL,1);	  break;	// stbd fuel tank
	}
	sensor_num++;
	if (sensor_num == MAX_SEND)
		sensor_num = 0;
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
			if (line_command == 'x')
			{
				if (byte == '!')
				{
					warning(0,"REBOOTING !!!",0);
					vTaskDelay(500 / portTICK_PERIOD_MS);
					ESP.restart();
					while (1) {}
				}
				else
				{

					line_command = 0;
					line_ptr = 0;
					in_line = 0;
					my_error("unexpected char after x==reboot",0);
					return;
				}
			}

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
				else if (line_command == 'r')
					handleCommand(PGN_ENGINE_RAPID,"rpms",linebuf,&random_rpms,&rpms);

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
			byte == 't' ||
			byte == 'r' ||
			byte == 'x' )
		{
			in_line = 1;
			line_ptr = 0;
			line_command = byte;
			if (byte == 'x')
				warning(0,"PRESS '!' to Confirm Reboot!!!",0);
		}
		else if (byte == 'p')
		{
			sendSensor(PGN_POSITION);
		}
		else if (byte == 'b')
		{
			show_bus = !show_bus;
			display(0,"SHOW_BUS <== %d",show_bus);
		}
		else if (byte == 'm')
		{
			show_sends = !show_sends;
			display(0,"SHOW_SEND_MESSAGES <== %d",show_sends);
		}
		else
		{
			warning(0,"unhandled serial character(%c) 0x%02x",byte>32?byte:' ',byte);
		}
	}
}



//--------------------------------------------
// low level canbus output debug display
//--------------------------------------------

#define DEBUG_MCP2515_OUT_LOW	0

#if DEBUG_MCP2515_OUT_LOW
	// overrides weakly linked prh_dbg_mcp2515_write() method
	// in #ifdef PRH_MODS in /libraries/CAN_BUS_Shield/mcp_can.cpp
	// to circular buffer the bytes written in the interrupt handler
	// and display() the in the main thread loop() method.
	// This was useful in the initial debugging of NMEA2000 to my Raymarine E80 MFD.

	#define MAX_CAN_MESSAGES	100

	typedef struct
	{
		byte len;
		byte first[4];
		byte buf[8];
	} dbg_msg_t;


	static volatile bool in_debug = 0;
	static int can_head = 0;
	static int can_tail = 0;
	static dbg_msg_t dbg_msgs[MAX_CAN_MESSAGES];


	void prh_dbg_mcp2515_write(byte *first, volatile const byte *buf, byte len)
	{
		int new_head = can_head  + 1;
		if (new_head >= MAX_CAN_MESSAGES)
			new_head = 0;
		if (new_head == can_tail)
		{
			my_error("DBG_CAN_MSG BUFFER OVERFLOW",0);
			return;
		}

		in_debug = 1;
		dbg_msg_t *msg = &dbg_msgs[can_head++];
		if (can_head >= MAX_CAN_MESSAGES)
			can_head = 0;

		msg->len = len;
		memcpy(msg->first,first,4);
		memcpy(msg->buf,(const void*) buf,len);
		in_debug = 0;
	}


	static void show_dbg_can_messages()
	{
		int head = can_head;
		if (in_debug)
			return;

		while (can_tail != head)
		{
			dbg_msg_t *msg = &dbg_msgs[can_tail++];
			if (can_tail >= MAX_CAN_MESSAGES)
				can_tail = 0;

			static char obuf[80];
			static int ocounter = 0;
			sprintf(obuf,"(%d) --> %02x%02x%02x%02x ",
				ocounter++,
				msg->first[0],
				msg->first[1],
				msg->first[2],
				msg->first[3]);

			int olen = strlen(obuf);
			for (int i=0; i<msg->len; i++)
			{
				sprintf(&obuf[olen],"%02x ",msg->buf[i]);
				olen += 3;
			}
			Serial.println(obuf);
		}
	}
#endif



//--------------------------------------------
// loop()
//--------------------------------------------

void loop()
{
	#if DEBUG_MCP2515_OUT_LOW
		show_dbg_can_messages();
	#endif

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
	

	#if ALIVE_LED
		static bool alive_on = 0;
		static uint32_t last_alive_time = 0;
		uint32_t alive_now = millis();
		uint32_t alive_delay = alive_on ? ALIVE_ON_TIME : ALIVE_OFF_TIME;
		if (alive_now - last_alive_time >= alive_delay)
		{
			alive_on = !alive_on;
			digitalWrite(ALIVE_LED,alive_on);
			last_alive_time = alive_now;
		}
	#endif
	
}	// loop()



