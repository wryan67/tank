// standard includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <byteswap.h>
#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <limits.h>
#include <limits>

// sound
#include <gst/gst.h>
#include <glib.h>

// engineering 
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

// devices
#include <ads1115rpi.h>
#include <mcp23x17rpi.h>
#include <pca9635rpi.h>
#include <pca9685.h>
#include <neopixel.h>  
#include <log4pi.h>

#include "mcp3008.h"


using namespace std;
using namespace common::utility;

bool doCalibration=false;
bool isMotorOn=false;

bool fireInTheHole=false;
bool interruptCharging=false;
bool cannonActivated=false;
bool movingTurret=false;

float cannonReleaseVoltage=46.4;  // was 46.5

Logger     logger("main");

/**************************************
 * 
 *  gstreamer
 * 
 *************************************/
GstElement *headset2tank;
GstElement *tank2headset;
GstElement *tank2headsetVolume;
GstElement *headset2tankVolume;
  
GMainLoop *loop;
GError *err = NULL;


int headsetDevice=2;
int tankSoundDevice=1;



/**************************************
 * 
 *  PCA9865   servo driver
 * 
 *************************************/
#define PCA9685_PIN_BASE 100

int PCA9865_CAP      = 50;
int PCA9865_RES      = 4046;
int PCA9865_ADDRESS  = 0x40;
int pca9685fd = -1;

int turretAspectControlChannel=0;

/**************************************
 * 
 *  R/C Receiver (mcp3008) channels
 * 
 *************************************/

// SPI Options
bool loadSPIDriver = false;
int  spiHandle = -1;
int  spiChannel = 0;
bool spiOverride = false;
int  spiSpeed = 500000;
int  channelType = MCP3008_SINGLE;
static volatile float mcp3008RefVolts = 1.23;
MCP3008DataType MCP3008Data[MCP3008_CHANNELS];
int  txs0108_oe=6;

int xChannel=7;
int yChannel=6;
int cannonElevationChannel=5;
int turretAspectChannel=4;

int fireChannel=3;  // right rocker
int push2talkChannel=2;
int hornChannel=1;
int leftDialChannel=0;


/**************************************
 * 
 *  ADS1115   a2d chip
 * 
 *************************************/
#define ADS1115_MaxBanks    2
#define ADS1115_MaxChannels 4

float ads1115Volts[ADS1115_MaxBanks][ADS1115_MaxChannels];

int ADS1115_ADDRESS1=0x49;
int a2dHandle1;

float vRef = 5.0;
int   gain = 4;
 
// 0x48 channels


// 0x49 channels
int cannonVoltsChannel=0;
int batteryChannel=1;
int fiveVSource=3;

float cannonVolts=0;

float xResolution, yResolution;
float xMiddle, yMiddle;

/**************************************
 * 
 *  PCA9635    motor speed controller &
 *             LED driver
 * 
 *************************************/


int pca9635Handle = -1;
int pca9635Address = 0x1f;

int rTrackChannel = 14;
int lTrackChannel = 15;
int joystickAlertLEDChannel=0;
int batteryAlertLEDChannel=2;

float minTurretAspectVoltage;
float idleTurretAspectVoltage;
float maxTurretAspectVoltage;


/**************************************
 * 
 *  MCP23017   motor direction controller
 * 
 *************************************/

int mcp23x17_handle   = -1;
int mcp23x17_address  = 0x20;
int mcp23x17_inta_pin = 7;
int mcp23x17_intb_pin = 0;

int xThrottle=0;
int yThrottle=1;

int xTurret=1;
int yTurret=0; // elevation

MCP23x17_GPIO lTrackReverse  = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 0);
MCP23x17_GPIO lTrackForward  = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 1);
MCP23x17_GPIO rTrackForward  = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 2);
MCP23x17_GPIO rTrackReverse  = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 3);

MCP23x17_GPIO cannonChargingPin  = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTA, 1);  // output
MCP23x17_GPIO cannonFirePin      = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTA, 0);  // output

MCP23x17_GPIO chargingTriggerPin   = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTA, 5);  // input
MCP23x17_GPIO cannonTriggerPin     = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTA, 6);  // input


enum directionType { stopped, forwardMotion, reverseMotion, turnLeft, turnRight, goStraight, undetermined};



/**************************************
 * 
 *  NeoPixel  battery indicator
 * 
 *************************************/

#define STRIP_TYPE              WS2812_WS2811_STRIP_RGB
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                18   // BCM numbering system
#define DMA                     10   // DMA=Direct Memory Access
int led_count =                 144;  // number of pixels in your led strip
                                     // If you have a matrix, you should use
                                     // the universal display driver as it 
                                     // supports neopixel matrices and 
                                     // drawing lines, circles, text, and 
                                     // even bmp images
                                     // https://github.com/wryan67/udd_rpi_lib

int  turretLED   = 0;
int  redColor    = 0xff0000;
int  greenColor  = 0x00ff00;
int  yellowColor = 0xffff00;

int  batteryPercent;
bool batteryWarning=true;
bool deadBattery=false;

/*--------------------------------------
 * 
 *  Subroutines
 * 
 *--------------------------------------
 */

unsigned int readMCP3008Channel(int channel)
{
    unsigned char buffer[3] = { 1, 0, 0 };
	buffer[1] = (channelType + channel) << 4;

    wiringPiSPIDataRW(spiChannel, buffer, 3);

	return ((buffer[1] & 3) << 8) + buffer[2];   
}
float getMCP3008Volts(int channel) {
    auto bits = readMCP3008Channel(channel);
	return ((bits)*mcp3008RefVolts) / 1024.0;
}


void setServoDutyCycle(int pin, int speed) {
	if (speed < 1) {
		speed = 0;
	} else if (speed >= 4096) {
		speed = 4096;
	}
	pwmWrite(PCA9685_PIN_BASE + pin, speed);
}


void delayedCannonShutdown() {
  usleep(30*1000*1000); // 30 second countdown
  while (fireInTheHole) {
    usleep(10*1000);
  }
  logger.info("disable cannon charging-auto");
  mcp23x17_digitalWrite(cannonChargingPin, HIGH);
  cannonActivated=false;
}


void fireCannon() {
  if (fireInTheHole) {
    return;
  } else {
    fireInTheHole=true;
  }
  usleep(2000);
  while (movingTurret) {
    usleep(500);
  }

  logger.info("powering cannon");
  mcp23x17_digitalWrite(cannonFirePin, HIGH);
  mcp23x17_digitalWrite(cannonChargingPin,LOW);

  while (cannonVolts<cannonReleaseVoltage && !interruptCharging) {
    usleep(2000);
  }
  if (interruptCharging) {
    interruptCharging=false;
    mcp23x17_digitalWrite(cannonChargingPin,HIGH);
    logger.info("charging interrupted");
    fireInTheHole=false;
    return;
  }

//  thread(delayedcannonShutdown).detach();
  logger.info("fire cannon");
  logger.info("disable cannon charging");
  mcp23x17_digitalWrite(cannonChargingPin, HIGH);
  usleep(250*1000); 

  mcp23x17_digitalWrite(cannonFirePin,LOW);
  usleep(200*1000); // 200 ms fire time;
  mcp23x17_digitalWrite(cannonFirePin,HIGH);

  logger.info("cannon discharged");
  fireInTheHole=false;
}

bool usage() {
    fprintf(stderr, "usage: tank\n");
    
    return false;
}



bool commandLineOptions(int argc, char **argv) {
	int c, index;

	while ((c = getopt(argc, argv, "a:cg:v:")) != -1)
		switch (c) {
			case 'a':
				sscanf(optarg, "%x", &ADS1115_ADDRESS1);
				break;
      case 'c':
        doCalibration=true;
        break;
			case 'g':
				sscanf(optarg, "%d", &gain);
				break;
			case 'v':
				sscanf(optarg, "%f", &vRef);
				break;
			case '?':
				if (optopt == 'm' || optopt=='t')
					fprintf(stderr, "Option -%c requires an argument.\n", optopt);
				else if (isprint(optopt))
					fprintf(stderr, "Unknown option `-%c'.\n", optopt);
				else
					fprintf(stderr,	"Unknown option character \\x%x.\n", optopt);

				return usage();
			default:
				abort();
		}

	
//	for (int index = optind; index < argc; index++)
//		printf("Non-option argument %s\n", argv[index]);
	return true;
}

struct rangeType {
  float min;
  float max;
  float avg;
  float breakpoints[50];
  float delta1;
  float delta2;
};

#define throttleChannelCount 2
#define turretChannelCount 2

int throttleChannels[throttleChannelCount] = {xChannel, yChannel};
int turretChannels[throttleChannelCount] = {turretAspectChannel, cannonElevationChannel};

rangeType idleThrottle[throttleChannelCount];
rangeType maxThrottle[throttleChannelCount];
rangeType minThrottle[throttleChannelCount];

rangeType idleTurret[turretChannelCount];
rangeType maxTurret[turretChannelCount];
rangeType minTurret[turretChannelCount];

int turretCenter=305;
int turretFullCW=turretCenter-210;  // 515
int turretFullCCW=turretCenter+210; //  95
int turretAspectDegree;


void getRange(rangeType range[], int xChannel, int yChannel, const char *message) {

  printf("%s",message);
  printf("\n xChannel=%d; yChannel=%d\n", xChannel, yChannel);
  getchar();

  int channels[2] = {xChannel, yChannel};


  for (int i=0;i<throttleChannelCount;++i) {
    range[i].min=numeric_limits<float>::max();
    range[i].max=numeric_limits<float>::min();
  }

  printf("calibrating"); fflush(stdout);
  long long now=currentTimeMillis();
  long long elapsed=0;
  while (elapsed<4500) {
    elapsed=currentTimeMillis()-now;
    float volts[4];
    if (elapsed%100==0) {
      printf("."); fflush(stdout);
    }


    int startTime=currentTimeMillis();
    for (int i=0;i<throttleChannelCount;++i) {
      float v=MCP3008Data[channels[i]].volts;
      if (v<0.1) {
        fprintf(stderr,"unable to detect joystick, is it on and paired?\n");
        exit(2);
      }

      // printf("g=%d; c=%d v=%7.4f\n",gain,i,v);

      if (v<range[i].min) range[i].min=v;
      if (v>range[i].max) range[i].max=v;

    }
  }

  for (int i=0;i<throttleChannelCount;++i) {
    float sum=range[i].min + range[i].max;
    float avg=sum / 2.0;
    range[i].avg = avg;

    range[i].delta1=abs(avg-range[i].min);
    range[i].delta2=abs(avg-range[i].max);
    
  }

  printf("\n");
}

void printRange(rangeType range[], const char *message) {
//  printf("type ------ X-Axis -------    ------ Y-Axis -------\n");
  printf("%-5s %6.4f %6.4f %6.4f     %6.4f %6.4f %6.4f\n", 
    message,
    range[xThrottle].min,      range[xThrottle].max,  range[xThrottle].avg,
    range[yThrottle].min,      range[yThrottle].max,  range[yThrottle].avg
  );
}

int calibrate() {

  int c1=throttleChannels[xThrottle];
  int c2=throttleChannels[yThrottle];
  
  printf("\n");
  getRange(idleThrottle, c1,c2, "center right joystick (throttle), then press enter");
  getRange(maxThrottle,  c1,c2, "  move right joystick (throttle) forward and right all the way, then press enter");
  getRange(minThrottle,  c1,c2, "  move right joystick (throttle) reverse and left all the way, then press enter");

  printf("\n");
  c1 = turretAspectChannel;
  c2 = cannonElevationChannel;
  getRange(idleTurret, c1,c2, "center left joystick (turret), then press enter");
  getRange(maxTurret,  c1,c2, "  move left joystick (turret), full cw (right), then press enter");
  getRange(minTurret,  c1,c2, "  move left joystick (turret), full ccw (left), then press enter");

  printf("     ------ X-Axis -------    ------ Y-Axis -------\n");
  printf("type     min    max    avg        min    max    avg\n");

  printRange(idleThrottle, "idle");
  printRange(minThrottle, "min");
  printRange(maxThrottle, "max");
  printf("\n");
  printRange(idleTurret, "idle");
  printRange(minTurret, "min");
  printRange(maxTurret, "max");
  

#define max(a,b) (a>b)?a:b

  float xDelta=0;
  xDelta=max(xDelta,idleThrottle[xThrottle].delta1);
  xDelta=max(xDelta,idleThrottle[xThrottle].delta2);
  xDelta=max(xDelta,minThrottle[xThrottle].delta1);
  xDelta=max(xDelta,minThrottle[xThrottle].delta2);
  xDelta=max(xDelta,maxThrottle[xThrottle].delta1);
  xDelta=max(xDelta,maxThrottle[xThrottle].delta2);

  float yDelta=0;
  yDelta=max(yDelta,idleThrottle[yThrottle].delta1);
  yDelta=max(yDelta,idleThrottle[yThrottle].delta2);
  yDelta=max(yDelta,minThrottle[yThrottle].delta1);
  yDelta=max(yDelta,minThrottle[yThrottle].delta2);
  yDelta=max(yDelta,maxThrottle[yThrottle].delta1);
  yDelta=max(yDelta,maxThrottle[yThrottle].delta2);

  xDelta = abs(maxThrottle[xThrottle].avg - minThrottle[xThrottle].avg)/4;
  yDelta = abs(maxThrottle[yThrottle].avg - minThrottle[yThrottle].avg)/4;

  float xMiddle = idleThrottle[xThrottle].avg;
  float yMiddle = idleThrottle[yThrottle].avg;

  printf("xDelta = %.4f; yDelta=%.4f\n",xDelta, yDelta);
  printf("center = (%.3f,%.3f)\n",xMiddle,yMiddle);
  fflush(stdout);

  char calibrationFilename[4096];
  char* HOME=getenv("HOME");  
  sprintf(calibrationFilename, "%s/.tank",HOME);
  
  FILE *calibrationFile = fopen(calibrationFilename, "w");
  if (!calibrationFile) {
    fprintf(stderr,"cannot open calibration file for writing\n");
    exit(2);
  }

  fprintf(calibrationFile,"throttle-resolution,%f,%f\n",xDelta,yDelta);
  fprintf(calibrationFile,"throttle-center,%f,%f\n",xMiddle,yMiddle);

  fprintf(calibrationFile, "throttle-x,%f,%f,%f\n",
              minThrottle[xThrottle].avg,
              idleThrottle[xThrottle].avg,
              maxThrottle[xThrottle].avg
  );

  fprintf(calibrationFile, "throttle-y,%f,%f,%f\n",
              minThrottle[yThrottle].avg,
              idleThrottle[yThrottle].avg,
              maxThrottle[yThrottle].avg
  );

  xMiddle = idleTurret[xTurret].avg;

  fprintf(calibrationFile,"turret-ccw,center,cw,%f,%f,%f\n", 
              minTurret[xTurret].avg,
              idleTurret[xTurret].avg,
              maxTurret[xTurret].avg
         );


  fflush(calibrationFile);
  fclose(calibrationFile);

  return 0;
}


void readCalibration() {
  float xDelta, yDelta;

  char calibrationFilename[4096];
  char* HOME=getenv("HOME");  
  sprintf(calibrationFilename, "%s/.tank",HOME);
  
  FILE *calibrationFile = fopen(calibrationFilename, "r");
  if (!calibrationFile) {
    fprintf(stderr,"system has not been calibrated, use -c option\n");
    exit(2);
  }

  char label[512];

  fscanf(calibrationFile,"throttle-resolution,%f,%f\n",&xDelta,&yDelta);
  fscanf(calibrationFile,"throttle-center,%f,%f\n",&xMiddle,&yMiddle);

  xResolution = xDelta + (xDelta * 0.1);
  yResolution = yDelta + (yDelta * 0.1);


  fscanf(calibrationFile, "throttle-x,%f,%f,%f\n",
              &minThrottle[xThrottle].avg,
              &idleThrottle[xThrottle].avg,
              &maxThrottle[xThrottle].avg
  );


  fscanf(calibrationFile, "throttle-y,%f,%f,%f\n",
              &minThrottle[yThrottle].avg,
              &idleThrottle[yThrottle].avg,
              &maxThrottle[yThrottle].avg
  );


  fscanf(calibrationFile,"turret-ccw,center,cw,%f,%f,%f\n", 
              &minTurretAspectVoltage,
              &idleTurretAspectVoltage,
              &maxTurretAspectVoltage
         );

  
  printf("calibration:\n");  
  printf("xDelta = %.4f; yDelta=%.4f\n",xDelta, yDelta);
  printf("xResolution = %.4f; yResolution=%.4f\n",xResolution, yResolution);
  printf("center = (%.3f,%.3f)\n",xMiddle,yMiddle);
}


void allStop() {

  mcp23x17_digitalWrite(lTrackForward, LOW);
  mcp23x17_digitalWrite(rTrackForward, LOW);
  mcp23x17_digitalWrite(lTrackReverse, LOW);
  mcp23x17_digitalWrite(rTrackReverse, LOW);

  logger.info("disable cannon charging-allStop");

  mcp23x17_digitalWrite(cannonChargingPin, HIGH);

  isMotorOn=false;
}


bool pca9635Init() {

  pca9635Handle = wiringPiI2CSetup(pca9635Address);
  if (pca9635Handle < 0) {
      fprintf(stderr,"Cannot initialize pca9635 on address 0x%02x\n",pca9635Address);
      return false;
  }


  int mode1 = 0x01;
  int mode2 = 0x04;

  wiringPiI2CWriteReg8(pca9635Handle, 0x00, mode1);
  wiringPiI2CWriteReg8(pca9635Handle, 0x01, mode2);

  wiringPiI2CWriteReg8(pca9635Handle, 0x14, 0xaa);
  wiringPiI2CWriteReg8(pca9635Handle, 0x15, 0xaa);
  wiringPiI2CWriteReg8(pca9635Handle, 0x16, 0xaa);
  wiringPiI2CWriteReg8(pca9635Handle, 0x17, 0xaa);
  delay(1);  // mandatroy 500 us delay when enabling pca9635 oscillator 

  int cmode1 = wiringPiI2CReadReg8(pca9635Handle, 0x00);
  int cmode2 = wiringPiI2CReadReg8(pca9635Handle, 0x01);

  if (cmode1 != mode1 || cmode2 != mode2) {
      fprintf(stderr,"pca9635 initialization failed\n");
      return false;
  }


  for (int led = 0; led < 16; ++led) {
      // pca9635DigitalWrite(pca9635Handle, led, 1);
    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 0);
  }

  return true;
}

void neopixel_colortest() {
    logger.info("moving color:  %6x", redColor);
    for (int i=0;i<4;++i) { 
      neopixel_setPixel(turretLED+i, redColor);
    }
    neopixel_render();
    delay(500);

    logger.info("braking color: %6x", yellowColor);
    for (int i=0;i<4;++i) { 
      neopixel_setPixel(turretLED+i, yellowColor);
    }
    neopixel_render();
    delay(500);

    logger.info("stopped color: %6x", greenColor);
    for (int i=0;i<4;++i) { 
      neopixel_setPixel(turretLED+i, greenColor);
    }
    neopixel_render();
    delay(500);
}

float batteryVolts=12.6;
bool isBatteryAlerting=false;

void batteryAlert() {

  if (isBatteryAlerting) {
    return;
  } else {
    isBatteryAlerting=true;
  }
  logger.info("battery alert");

  long c=0;
  while (batteryVolts<11.4) {
    if ((++c)%2==0) {
      pca9635SetDutyCycle(pca9635Handle, batteryAlertLEDChannel, 50);
    } else {
      pca9635SetDutyCycle(pca9635Handle, batteryAlertLEDChannel, 5);
    }
    usleep(100*1000);
  }

  pca9635SetDutyCycle(pca9635Handle, batteryAlertLEDChannel, 0);
  isBatteryAlerting=false;
}

void batteryCheck() {
  pca9635SetDutyCycle(pca9635Handle, batteryAlertLEDChannel, 0);

  int readCount = 250;
  while (true) {
    float totalVolts=0;
    for (int i=0;i<readCount;++i) {
        usleep(10*1000);
        float volts = ads1115Volts[0][batteryChannel];
        float batteryVolts=((5100+200)*volts)/200;
        // logger.info("--ads volts=%f; battery volts: %6.3f", volts, batteryVolts); 

        while (volts<0.1 || volts>0.51|| isMotorOn) {
          if (volts<0.1) {
            usleep(10*1000);
          } else {
            usleep(100*1000);
          }
          volts = ads1115Volts[0][batteryChannel];
        }
        totalVolts+=volts;
    }
    batteryVolts=((5100+200)*(totalVolts/readCount))/200;
    logger.info("battery volts: %6.3f", batteryVolts); 
    fflush(stdout); fflush(stderr);

    if (batteryVolts>11.3) {
      deadBattery=false;
    } else if (batteryVolts>11.2) {
      batteryWarning=true;
      deadBattery=false;
    } else {
      deadBattery=true;
    }
  }
}

void neopixel_setup() {
    int ledType = WS2811_STRIP_RGB;
    int ret=neopixel_init(ledType, WS2811_TARGET_FREQ, DMA, GPIO_PIN, led_count);

    if (ret!=0) {
        fprintf(stderr, "neopixel initialization failed: %s\n", neopixel_error(ret));
        exit(5);
    }

    neopixel_setBrightness(32);
    neopixel_clear();

    for (int i=0;i<4;++i) { 
      neopixel_setPixel(turretLED+i, redColor);
    }
    neopixel_render();

    // neopixel_colortest();

}

bool triggerActivated=false;
void triggerAction(int value) {
  logger.info("cannon trigger value=%d",value);
  if (value==0) {
    mcp23x17_digitalWrite(cannonFirePin, LOW);
  } else {
    interruptCharging=true;
    mcp23x17_digitalWrite(cannonFirePin, HIGH);
  }
  triggerActivated=false;
}

void cannonTriggerActivated(MCP23x17_GPIO gpio, int value) {
  if (!triggerActivated) {
    triggerActivated=true;

    thread(triggerAction,value).detach();
  }
}


bool chargingActivated=false;
void chargingAction(int value) {
  logger.info("charging charging value=%d",value);
  if (value==0) {
    mcp23x17_digitalWrite(cannonChargingPin, LOW);
  } else {
      logger.info("disable cannon charging-action-pin");

    mcp23x17_digitalWrite(cannonChargingPin, HIGH);
  }
  chargingActivated=false;
}

void cannonChargingActivated(MCP23x17_GPIO gpio, int value) {
  if (!chargingActivated) {
    chargingActivated=true;

    thread(chargingAction,value).detach();
  }
}


void readAnalogChannels(int bank, int handle, int gain) {
  while (true) {
    for (int i=0;i<ADS1115_MaxChannels;++i) {
      ads1115Volts[bank][i]=readVoltageSingleShot(handle, i, gain);
    }
  }
}

float percentDifference(float a, float b) {
  return ( abs(a-b)/ ((a+b)/2) ) * 100.0;
}

void readMCP3008Channels() {
  logger.info("thread started--read mcp3008 channels");
  while (true) {
    for (int i=0;i<MCP3008_CHANNELS;++i) {
      MCP3008Data[i].volts=getMCP3008Volts(i);
      if (MCP3008Data[i].window.size()>20) {
        MCP3008Data[i].window.erase(MCP3008Data[i].window.begin());
      }
      MCP3008Data[i].window.push_back(MCP3008Data[i].volts);
      float sum=0;
      for (float volts : MCP3008Data[i].window) {
        sum+=volts;
      }
      MCP3008Data[i].movingAverage=sum/MCP3008Data[i].window.size();
    }
    usleep(1000);
  }
}

int getTurretAspect(int dutyCycle) {

  float b1 = turretFullCW-turretFullCCW+1;
  float p  = dutyCycle - turretFullCCW;
  float a3 = p / b1;

  return a3 * 180 - 90;

}


void turretAspect() {
  int dutyCycle=turretCenter;
  int lastCycle=0;

  logger.info("minTurretAspectVoltage:  %f",minTurretAspectVoltage);
  logger.info("maxTurretAspectVoltage:  %f",maxTurretAspectVoltage);
  logger.info("turretFullCCW-CW:        <%d,%d>",turretFullCCW, turretFullCW);

  while (true) {
    if (fireInTheHole) {
      usleep(25*1000);
      continue;
    } else {
      usleep(2*1000);
    }

    float volts=MCP3008Data[turretAspectChannel].volts;

    float pDiff=abs(percentDifference(volts,MCP3008Data[turretAspectChannel].movingAverage));
    if (pDiff>5) {
      continue;
    }


    float a1 = maxTurretAspectVoltage-minTurretAspectVoltage;
    float a2 = volts - minTurretAspectVoltage;
    float a3 = a2/a1;
    float b1 = turretFullCW-turretFullCCW+1;

    dutyCycle = a3*b1+turretFullCCW;

    int userAspect = getTurretAspect(dutyCycle);



    if (abs(dutyCycle-lastCycle)>4 && !fireInTheHole) {
      movingTurret=true;
      lastCycle=dutyCycle;
      if (abs(userAspect)<16) {
        setServoDutyCycle(turretAspectControlChannel, turretCenter);
        if (turretAspectDegree!=turretCenter) {
          turretAspectDegree=turretCenter;
          logger.info("turret aspect volts=%f; aspect: %d delta: %5.2f%%  mvAvg: %4.2f", volts, userAspect, pDiff, MCP3008Data[turretAspectChannel].movingAverage);
        }
      } else {
        turretAspectDegree=dutyCycle;
        setServoDutyCycle(turretAspectControlChannel, dutyCycle);
        logger.info("turret aspect volts=%f;  aspect: %d delta: %5.2f%% mvAvg: %4.2f", volts, userAspect, pDiff, MCP3008Data[turretAspectChannel].movingAverage);
      }


      movingTurret=false;
    }
  }
}

float lastCannonVolts=0;
void turretColor() {
  int maxBrighness=250;
  int brightness=230;
  int bDelta=200;

  long c;
  while (true) {
    float adsVolts=ads1115Volts[0][cannonVoltsChannel];
    cannonVolts=adsVolts*100;

    float percent = cannonVolts/50;
    float greenPercent=percent;

    uint32_t color = (int)(percent*maxBrighness) << 8;

    float redPercent=0;
    if (percent>0.50) {
      redPercent=(percent-.5)/.5;
      int red = (int)(redPercent *maxBrighness) << 16;
      color = red;
      brightness-=bDelta*(1-redPercent);

      greenPercent=0.8-redPercent;
      if (greenPercent<0.1) {
        greenPercent=0;
      }
      int green = (int(greenPercent *maxBrighness)) << 8;
      brightness+=bDelta*(1-redPercent);


      color |= green;
    }

    for (int i=0;i<4;++i) { 
      neopixel_setPixel(turretLED+i, color);
    }
    neopixel_setBrightness(brightness);
    neopixel_render();

    usleep(25*1000);
    // if ((++c)%250==0) {
    //   if (cannonVolts != lastCannonVolts) {        
    //     lastCannonVolts=cannonVolts;
    //     logger.info("adsVolts=%f; volts=%f; percent=%f greenPercent=%f redPercent=%f",adsVolts,cannonVolts, percent, greenPercent, redPercent);
    //   }
    // }
  }
}

void muteTank(bool mute);

directionType direction=undetermined;
directionType trackAction=goStraight;
int dd=-1;

void throttleControl() {
//@@
  float xVolts = MCP3008Data[throttleChannels[xThrottle]].volts;
  float yVolts = MCP3008Data[throttleChannels[yThrottle]].volts;

  float xMin=minThrottle[xThrottle].avg;
  float xMax=maxThrottle[xThrottle].avg;
  float yMin=minThrottle[yThrottle].avg;
  float yMax=maxThrottle[yThrottle].avg;
  
  float xPercent =   (xVolts-xMin)/(xMax-xMin);
  float yPercent = 1-(yVolts-yMax)/(yMin-yMax);

  if (yPercent<0.03) yPercent=0;
  if (xPercent<0.03) xPercent=0;
  if (yPercent>0.95) yPercent=1;
  if (xPercent>0.95) xPercent=1;




  xPercent=(xPercent-0.5)*2;
  yPercent=(yPercent-0.5)*2;

  if (abs(xPercent)<0.08) xPercent=0;
  if (abs(yPercent)<0.08) yPercent=0;

  // if (xPercent>0.95) xPercent=1;
  // if (yPercent<0.95) yPercent=-1;


  // printf("<%5.3f,%5.3f> <%5.2f,%5.2f>\r", xVolts, yVolts, xPercent, yPercent);
// return;

  float lThrottle=0;
  float rThrottle=0;



  if (yPercent==0 && xPercent==0) {
    if (dd!=1) {
      dd=1;
      muteTank(false);
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "stop-100 dd=1");
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 255);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 255);
      mcp23x17_digitalWrite(lTrackForward, LOW);
      mcp23x17_digitalWrite(rTrackForward, LOW);
      mcp23x17_digitalWrite(lTrackReverse, LOW);
      mcp23x17_digitalWrite(rTrackReverse, LOW);
    }
  } else if (yPercent>abs(xPercent) || yPercent>=0.95) {  
    int lp = (255-abs(yPercent*255));
    int rp = (255-abs(yPercent*255));


    if (xPercent<0) {
      lp+=abs(xPercent*255);
    } else {
      rp+=abs(xPercent*255);
    }

    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, lp);
    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, rp);
    if (dd!=2) {
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> [%3d,%3d] %s", 
          xVolts, yVolts, xPercent, yPercent, 255-lp,255-rp, "move forward dd=2");

      dd=2;
      muteTank(true);

      mcp23x17_digitalWrite(lTrackForward, HIGH);
      mcp23x17_digitalWrite(rTrackForward, HIGH);
      mcp23x17_digitalWrite(lTrackReverse, LOW);
      mcp23x17_digitalWrite(rTrackReverse, LOW);
    }
  } else if (yPercent<-abs(xPercent) || yPercent<=-0.95) { 
    int lp = (255-abs(yPercent*255));
    int rp = (255-abs(yPercent*255));

    if (xPercent<0) {
      lp+=abs(xPercent*255);
    } else {
      rp+=abs(xPercent*255);
    }
    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, lp);
    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, rp);

    if (dd!=3) {
      dd=3;
      muteTank(true);
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "move backward; dd=3");
      mcp23x17_digitalWrite(lTrackForward, LOW);
      mcp23x17_digitalWrite(rTrackForward, LOW);
      mcp23x17_digitalWrite(lTrackReverse, HIGH);
      mcp23x17_digitalWrite(rTrackReverse, HIGH);     
    }
  } else if (xPercent>abs(yPercent)) { // turn right


      int left  = 255-abs(xPercent*255);
      int right = 255-abs(xPercent*255) + abs(yPercent*255);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, left);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, right);

      if (dd!=4) {
        dd=4;
        muteTank(true);
        logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> [%3d,%3d] %s", 
          xVolts, yVolts, xPercent, yPercent, left,right, "turn right; dd=4");

        mcp23x17_digitalWrite(lTrackForward, LOW);
        mcp23x17_digitalWrite(rTrackForward, HIGH);
        mcp23x17_digitalWrite(lTrackReverse, HIGH);
        mcp23x17_digitalWrite(rTrackReverse, LOW);
      }
  } else if (xPercent<-abs(yPercent)) { // turn left
      int right = 255-abs(xPercent*255);
      int left  = 255-abs(xPercent*255) + abs(yPercent*255);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, left);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, right);

      if (dd!=5) {
        dd=5;
        muteTank(true);
        logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> [%3d,%3d] %s", 
          xVolts, yVolts, xPercent, yPercent, left,right, "turn left; dd=5");

        mcp23x17_digitalWrite(lTrackForward, HIGH);
        mcp23x17_digitalWrite(rTrackForward, LOW);
        mcp23x17_digitalWrite(lTrackReverse, LOW);
        mcp23x17_digitalWrite(rTrackReverse, HIGH);
      }

  } else {
    if (dd!=6) {
      dd=6;
      muteTank(false);
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "stop-200; dd=6");
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 255);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 255);
      mcp23x17_digitalWrite(lTrackForward, LOW);
      mcp23x17_digitalWrite(rTrackForward, LOW);
      mcp23x17_digitalWrite(lTrackReverse, LOW);
      mcp23x17_digitalWrite(rTrackReverse, LOW);
    }
  }


}

void throttleControlOld(long long &now) {
    float volts[4];


    for (int i=0;i<throttleChannelCount;++i) {
      volts[i]=MCP3008Data[throttleChannels[i]].volts;
    }

    // printf("%lld %12.6f %12.6f %12.6f %12.6f\r", 
    //         now, volts[0], volts[1], volts[2], volts[3]);


    if (volts[yThrottle]>0.1) {
      if (volts[yThrottle]<yMiddle-yResolution) {         // move forward
        if (direction != forwardMotion) {
          direction=forwardMotion;
          isMotorOn=true;
          mcp23x17_digitalWrite(lTrackForward, HIGH);
          mcp23x17_digitalWrite(rTrackForward, HIGH);
          mcp23x17_digitalWrite(lTrackReverse, LOW);
          mcp23x17_digitalWrite(rTrackReverse, LOW);
      
          const char *notes = "moving forward";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
        }
        // allStop();
        // exit(2);
      } else if (volts[yThrottle]>yMiddle+yResolution) {  // move backward
        if (direction != reverseMotion) {
          direction = reverseMotion;
          isMotorOn=true;
          mcp23x17_digitalWrite(lTrackForward, LOW);
          mcp23x17_digitalWrite(rTrackForward, LOW);
          mcp23x17_digitalWrite(lTrackReverse, HIGH);
          mcp23x17_digitalWrite(rTrackReverse, HIGH);

          const char *notes = "moving backward";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
        }
        // allStop();
        // exit(2);
      } else {                                           // stop
        if (direction!=stopped) {
          direction=stopped;
          const char *notes = "stopped";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
          allStop();
        }
      }

      if (volts[xThrottle]>xMiddle+xResolution) {         // turn right
        if (trackAction!=turnRight) {
          trackAction=turnRight;
          const char *notes = "turn right";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 0);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 255);
        }
      } else if (volts[xThrottle]<xMiddle-xResolution) {  // turn left
        if (trackAction!=turnLeft) {
          trackAction=turnLeft;
          const char *notes = "turn left";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 255);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 0);

          printf("volts[x]: %6.4f \n",volts[xThrottle]);
          printf("xMiddle:  %6.4f %6.4f\n",xMiddle, xResolution);
          printf("xTarget:  %6.4f \n",xMiddle-xResolution);
          // exit(2);
        }
      } else {                                            // go straight
        if (trackAction!=goStraight) {
          trackAction=goStraight;
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 0);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 0);
        }
      }
    }

}

bool setupMCP3008() {


	// The speed parameter is an integer in the range 500,000 through 32,000,000 and represents the SPI clock speed in Hz


	if ((spiHandle = wiringPiSPISetup(spiChannel, spiSpeed)) < 0)
	{
		fprintf(stderr, "opening SPI bus failed: %s\n", strerror(errno));
		return false;
	}

  double volts=0.0;
  for (int i=0;i<MCP3008_CHANNELS;++i) {
    volts+=getMCP3008Volts(i);
  }

  if (volts==0.0) {
    fprintf(stderr, "mcp3008 not detected\n");
    return false;
  }

	return true;
}


gboolean gst_bus_call (GstBus *bus, GstMessage *msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;

  switch (GST_MESSAGE_TYPE (msg)) {

    case GST_MESSAGE_EOS:
      g_main_loop_quit (loop);
      break;

    case GST_MESSAGE_ERROR: {
      gchar  *debug;
      GError *error;

      gst_message_parse_error (msg, &error, &debug);
      g_free (debug);

      g_printerr ("Error: %s\n", error->message);
      g_error_free (error);

      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }

  return TRUE;
}
void stopAudio() {
  gst_element_set_state(headset2tank, GST_STATE_NULL); 
  gst_element_set_state(tank2headset, GST_STATE_NULL); 
}

void playTank2Headset() {
  stopAudio();
  gst_element_set_state(tank2headset, GST_STATE_PLAYING);
}
void playHeadset2Tank() {
  stopAudio();
  gst_element_set_state(headset2tank, GST_STATE_PLAYING);
}


void muteTank(bool mute) {
  float volume=(mute)?0.1:1.0;

  if (!mute) {
    usleep(500*1000);
  }
  g_object_set(tank2headsetVolume, "volume", volume, NULL); 
}


void playFile(const char *filename,float volume) {
  char tmpstr[4096];
  GstElement *dixie;
  GstBus *bus;

  sprintf(tmpstr,"filesrc location=%s ! decodebin ! audioconvert ! audioresample ! volume volume=%f ! level ! alsasink device=dmix:%d,0",filename,volume,tankSoundDevice);


  playHeadset2Tank();
   
  dixie = gst_parse_launch(tmpstr, &err);
  gst_element_set_state(dixie, GST_STATE_PLAYING);  
  bus = gst_element_get_bus(dixie);
  
  gst_bus_add_watch (bus, gst_bus_call, loop);
  g_main_loop_run(loop);
gst_element_set_state(dixie, GST_STATE_NULL);  
  
  playTank2Headset();
}


void gst_init(int argc, char *argv[]) {
  char tmpstr[4096];

    gst_init(&argc, &argv);
  
  loop = g_main_loop_new(NULL, FALSE);

  sprintf(tmpstr,"alsasrc device=hw:%d,0 ! audioconvert ! audioresample ! volume volume=1.0 name=tank2headsetVolume ! alsasink device=dmix:%d,0",tankSoundDevice,headsetDevice);
  tank2headset = gst_parse_launch(tmpstr, &err);

  sprintf(tmpstr,"alsasrc device=hw:%d,0 ! audioconvert ! audioresample ! volume volume=1.0 name=headset2tankVolume ! alsasink device=dmix:%d,0",headsetDevice,tankSoundDevice);
  headset2tank = gst_parse_launch(tmpstr, &err);

  tank2headsetVolume = gst_bin_get_by_name (GST_BIN(tank2headset), "tank2headsetVolume");
  headset2tankVolume = gst_bin_get_by_name (GST_BIN(headset2tank), "headset2tankVolume");
  g_assert(tank2headsetVolume);
  g_assert(headset2tankVolume);

  playTank2Headset();

  logger.info("gst initialized");
}



void push2talk() {
  logger.info("push2talk activated");
  bool talking=true;
  
  while (true) {
    usleep(10*1000);
    float talkVolts=MCP3008Data[push2talkChannel].volts;

    // logger.info("push2talk volts: %5.2f",talkVolts);

    if (talkVolts>0.28) {
      // talking
      if (!talking) {
        playHeadset2Tank();
        talking=true;
      }

    } else {
      // listening
      if (talking) {
        playTank2Headset();
        talking=false;
      }
    }

    if (MCP3008Data[hornChannel].volts>0.35) {
      playFile("/home/wryan/sounds/dixie.mp3",0.3);
    }
    if (MCP3008Data[hornChannel].volts<0.25) {
      playFile("/home/wryan/sounds/ahooga.mp3",0.3);
    }
  } 
}

void knobtest() {

  while (true) {
    for (int i=0;i<MCP3008_CHANNELS;++i) {
      printf("%d: %5.3f | ", i, MCP3008Data[i].volts);
    }
    printf("\r");
    delay(100);
  }

}

int main(int argc, char **argv) {  
  
  if (!commandLineOptions(argc, argv)) {
    return 1;
  }
  if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }

  gst_init(argc,argv);

  if ((pca9685fd=pca9685Setup(PCA9685_PIN_BASE, PCA9865_ADDRESS, PCA9865_CAP)) <= 0) {
		printf("pca9685 setup failed!\n");
		return false;
	}


  printf("use -h to get help on command line options\n");
  printf("accessing ads1115 chip on i2c address 0x%02x\n", ADS1115_ADDRESS1);


/*    mcp23017    */
  mcp23x17_handle = mcp23x17_setup(0, mcp23x17_address, mcp23x17_inta_pin, mcp23x17_intb_pin);
  if (mcp23x17_handle < 0) {
      fprintf(stderr, "mcp23017 could not be initialized\n");
      return 9;
  }

  mcp23x17_setDebug(false);
  mcp23x17_setPinOutputMode(lTrackForward, LOW);
  mcp23x17_setPinOutputMode(rTrackForward, LOW);
  mcp23x17_setPinOutputMode(lTrackReverse, LOW);
  mcp23x17_setPinOutputMode(rTrackReverse, LOW);
  
  mcp23x17_setPinOutputMode(cannonChargingPin, HIGH);
  mcp23x17_setPinOutputMode(cannonFirePin,     HIGH);

  mcp23x17_setPinInputMode(cannonTriggerPin,   true, cannonTriggerActivated);
  mcp23x17_setPinInputMode(chargingTriggerPin, true, cannonChargingActivated);


/*    pca9535    */
  if (!pca9635Init()) {
    return 2;
  }

  pinMode(txs0108_oe,OUTPUT);
  digitalWrite(txs0108_oe,1);

	if (!setupMCP3008()) {
		printf("mcp3008 setup failed\n");
		exit(2);
	}

   thread(readMCP3008Channels).detach();
   usleep(100*1000);

// knobtest


/*    ADS11115    */
// o = operation mode
// x = mux
// g = gain
// m = mode
//                oxxx gggm
// default 0x8583 1000 0101 1000 0011
//                1111 0101 1000 0011
//                1111 0101 1000 0011
  a2dHandle1 = getADS1115Handle(ADS1115_ADDRESS1);
  float max=getADS1115MaxGain(gain);

  if (doCalibration) {
    calibrate();
  }
  
  readCalibration();
  thread(readAnalogChannels,0,a2dHandle1,gain).detach();
  thread(push2talk).detach();


  neopixel_setup();

  logger.info("joystick check");
  long c=0;
  int  loop=0;

  while (MCP3008Data[yChannel].volts<0.1) {
    if (++c%10==0) ++loop;
 
    pca9635SetDutyCycle(pca9635Handle, joystickAlertLEDChannel, 50*(loop%2));
    usleep(10*1000);
  }
  pca9635SetDutyCycle(pca9635Handle, joystickAlertLEDChannel, 50);


  logger.info("battery check");
  thread(batteryCheck).detach();

  float lastVolts[4]={0,0,0,0};
  float volts[4]={0,0,0,0};

  thread(turretAspect).detach();
  thread(turretColor).detach();

  while (true) {
    if (deadBattery) {
      allStop();
      logger.info("dead battery");
      batteryAlert();
    }

    float fireVolts=MCP3008Data[fireChannel].volts;

    if (fireVolts>0.32 && !fireInTheHole) {
      logger.info("fireVolts=%12.6f", fireVolts);
      thread(fireCannon).detach();
      usleep(2000);
    }
    

    throttleControl();

    fflush(stdout);
  }


}

