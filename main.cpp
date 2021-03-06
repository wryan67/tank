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



#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <ads1115rpi.h>
#include <mcp23x17rpi.h>
#include <pca9635rpi.h>
#include <pca9685.h>
#include <neopixel.h>  
#include <log4pi.h>



using namespace std;
using namespace common::utility;

bool doCalibration=false;
bool isMotorOn=false;

bool fireInTheHole=false;
bool interruptCharging=false;
bool cannonActivated=false;
bool movingTurret=false;

Logger     logger("main");

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
 *  ADS1115   a2d chips
 * 
 *************************************/
#define ADS1115_MaxBanks    2
#define ADS1115_MaxChannels 4

float ads1115Volts[ADS1115_MaxBanks][ADS1115_MaxChannels];

int ADS1115_ADDRESS1=0x48;
int ADS1115_ADDRESS2=0x49;
int a2dHandle1;
int a2dHandle2;

float vRef = 5.0;
int   gain = 4;
 
// 0x48 channels
int xChannel=0;
int yChannel=1;
int turretAspectChannel=3;
int cannonElevationChannel=2;

// 0x49 channels
int cannonVoltsChannel=0;
int batteryChannel=1;
int fireChannel=2;
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

  while (cannonVolts<46.5 && !interruptCharging) {
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


void getRange(rangeType range[], int xChannel, int yChannel, const char *message) {
  printf("%s",message);
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
      float v=readVoltageSingleShot(a2dHandle1, channels[i], gain);

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

  int c1=throttleChannels[xChannel];
  int c2=throttleChannels[yChannel];
  
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
        float volts = ads1115Volts[1][batteryChannel];
        float batteryVolts=((5100+200)*volts)/200;
        // logger.info("--ads volts=%f; battery volts: %6.3f", volts, batteryVolts); 

        while (volts<0.1 || volts>0.51|| isMotorOn) {
          if (volts<0.1) {
            usleep(10*1000);
          } else {
            usleep(100*1000);
          }
          volts = ads1115Volts[1][batteryChannel];
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
      usleep(1000);
    }

    float volts=ads1115Volts[0][turretAspectChannel];

    float a1 = maxTurretAspectVoltage-minTurretAspectVoltage;
    float a2 = volts - minTurretAspectVoltage;
    float a3 = a2/a1;
    float b1 = turretFullCW-turretFullCCW+1;

    dutyCycle = a3*b1+turretFullCCW;


    if (abs(dutyCycle-lastCycle)>5 && !fireInTheHole) {
      movingTurret=true;
      lastCycle=dutyCycle;
      setServoDutyCycle(turretAspectControlChannel, dutyCycle);
      float dutyCyclePercent=(float)dutyCycle/4096;
      logger.info("turret aspect volts=%f;  duty cycle: %5.2f  fireInTheHole: %d", volts, dutyCyclePercent*100, fireInTheHole);
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
    float adsVolts=ads1115Volts[1][cannonVoltsChannel];
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

directionType direction=undetermined;
directionType trackAction=goStraight;
int dd=-1;

void throttleControl() {
//@@
  float xVolts = ads1115Volts[0][throttleChannels[xChannel]];
  float yVolts = ads1115Volts[0][throttleChannels[yChannel]];

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
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "stop-100");
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrackChannel, 255);
      wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrackChannel, 255);
      mcp23x17_digitalWrite(lTrackForward, LOW);
      mcp23x17_digitalWrite(rTrackForward, LOW);
      mcp23x17_digitalWrite(lTrackReverse, LOW);
      mcp23x17_digitalWrite(rTrackReverse, LOW);
    }
  } else if (yPercent>abs(xPercent) || yPercent>=0.95) {  // move forward
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
          xVolts, yVolts, xPercent, yPercent, 255-lp,255-rp, "move forward");

      dd=2;
      mcp23x17_digitalWrite(lTrackForward, HIGH);
      mcp23x17_digitalWrite(rTrackForward, HIGH);
      mcp23x17_digitalWrite(lTrackReverse, LOW);
      mcp23x17_digitalWrite(rTrackReverse, LOW);
    }
  } else if (yPercent<-abs(xPercent) || yPercent<=-0.95) { // move backward
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
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "move backward");
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
        logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> [%3d,%3d] %s", 
          xVolts, yVolts, xPercent, yPercent, left,right, "turn right");

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
        logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> [%3d,%3d] %s", 
          xVolts, yVolts, xPercent, yPercent, left,right, "turn left");

        mcp23x17_digitalWrite(lTrackForward, HIGH);
        mcp23x17_digitalWrite(rTrackForward, LOW);
        mcp23x17_digitalWrite(lTrackReverse, LOW);
        mcp23x17_digitalWrite(rTrackReverse, HIGH);
      }

  } else {
    if (dd!=6) {
      dd=6;
      logger.info("<%5.3f,%5.3f> <%5.2f,%5.2f> %s", 
          xVolts, yVolts, xPercent, yPercent, "stop-200");
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
      volts[i]=ads1115Volts[0][throttleChannels[i]];
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

int main(int argc, char **argv)
{  
  if (!commandLineOptions(argc, argv)) {
    return 1;
  }
  if (wiringPiSetup()!=0) {
    printf("cannot initialize WiringPi\n");
    return 1;
  }


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
  a2dHandle2 = getADS1115Handle(ADS1115_ADDRESS2);
  float max=getADS1115MaxGain(gain);

  if (doCalibration) {
    calibrate();
  }
  readCalibration();

  thread(readAnalogChannels,0,a2dHandle1,gain).detach();
  thread(readAnalogChannels,1,a2dHandle2,gain).detach();
 

  neopixel_setup();

  logger.info("joystick check");
  long c=0;
  int  loop=0;

  while (ads1115Volts[0][yChannel]<0.1) {
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

    float fireVolts=ads1115Volts[1][fireChannel];

    if (fireVolts>0.32 && !fireInTheHole) {
      logger.info("fireVolts=%12.6f", fireVolts);
      thread(fireCannon).detach();
      usleep(2000);
    }
    

    throttleControl();

    fflush(stdout);
  }


}

