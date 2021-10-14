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


using namespace std;

bool doCalibration=false;

int ADS1115_ADDRESS=0x48;
int ADS1115_HANDLE;

float vRef = 5.0;
int   gain = 4;

int pca9635Handle = -1;
int pca9635Address = 0x1f;

int rTrack = 15;
int lTrack = 14;

int mcp23x17_handle = -1;
int mcp23x17_address = 0x20;
int mcp23x17_inta_pin = 2;
int mcp23x17_intb_pin = 3;

int xChannel=0;
int yChannel=1;

int xThrottle=0;
int yThrottle=1;


MCP23x17_GPIO lTrackReverse = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 0);
MCP23x17_GPIO lTrackForward = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 1);
MCP23x17_GPIO rTrackForward = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 2);
MCP23x17_GPIO rTrackReverse = mcp23x17_getGPIO(mcp23x17_address, MCP23x17_PORTB, 3);

enum directionType { stopped, forward, reverse, turnLeft, turnRight, goStraight, undetermined};



unsigned long long currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (unsigned long long)(currentTime.tv_sec) * 1000 +
        (unsigned long long)(currentTime.tv_usec) / 1000;
}

bool usage() {
    fprintf(stderr, "usage: knobtest [-a address] [-g gain] [-v vRef]\n");
    fprintf(stderr, "a = hex address of the ads1115 chip\n");
    fprintf(stderr, "v = refrence voltage\n");
    fprintf(stderr, "g = gain; default=0; see chart:\n");
    fprintf(stderr, "    0 = +/- %5.3f volts\n", 6.144);
    fprintf(stderr, "    1 = +/- %5.3f volts\n", 4.096);
    fprintf(stderr, "    2 = +/- %5.3f volts\n", 2.048);
    fprintf(stderr, "    3 = +/- %5.3f volts\n", 1.024);
    fprintf(stderr, "    4 = +/- %5.3f volts\n", 0.512);
    fprintf(stderr, "    5 = +/- %5.3f volts\n", 0.256);

    return false;
}



bool commandLineOptions(int argc, char **argv) {
	int c, index;

	while ((c = getopt(argc, argv, "a:cg:v:")) != -1)
		switch (c) {
			case 'a':
				sscanf(optarg, "%x", &ADS1115_ADDRESS);
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
int throttleChannels[throttleChannelCount] = {xChannel, yChannel};

rangeType idleThrottle[throttleChannelCount];
rangeType maxThrottle[throttleChannelCount];
rangeType minThrottle[throttleChannelCount];

void getRange(rangeType range[], const char *message) {
  printf("%s",message);
  getchar();


  for (int i=0;i<throttleChannelCount;++i) {
    range[i].min=numeric_limits<float>::max();
    range[i].max=numeric_limits<float>::min();
  }

  printf("calibrating"); fflush(stdout);
  long long now=currentTimeMillis();
  long long elapsed=0;
  while (elapsed<5000) {
    elapsed=currentTimeMillis()-now;
    float volts[4];
    if (elapsed%100==0) {
      printf("."); fflush(stdout);
    }


    int startTime=currentTimeMillis();
    for (int i=0;i<throttleChannelCount;++i) {
      float v=readVoltageSingleShot(ADS1115_HANDLE, throttleChannels[i], gain);

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

  printf("\n");
  getRange(idleThrottle, "center joystick, then press enter");
  getRange(maxThrottle, "move joystick forward and right all the way, then press enter");
  getRange(minThrottle, "move joystick reverse and left all the way, then press enter");

  printf("     ------ X-Axis -------    ------ Y-Axis -------\n");
  printf("type     min    max    avg        min    max    avg\n");

  printRange(idleThrottle, "idle");
  printRange(minThrottle, "min");
  printRange(maxThrottle, "max");

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
  
  fflush(calibrationFile);
  fclose(calibrationFile);

  return 0;
}

float xResolution, yResolution;
float xMiddle, yMiddle;

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
    wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrack, 0);
  }

  return true;
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

  printf("use -h to get help on command line options\n");
  printf("accessing ads1115 chip on i2c address 0x%02x\n", ADS1115_ADDRESS);
  ADS1115_HANDLE = getADS1115Handle(ADS1115_ADDRESS);

  mcp23x17_handle = mcp23x17_setup(0, mcp23x17_address, mcp23x17_inta_pin, mcp23x17_intb_pin);
  if (mcp23x17_handle < 0) {
      fprintf(stderr, "mcp23017 could not be initialized\n");
      return 9;
  }

  mcp23x17_setDebug(false);

  if (!pca9635Init()) {
    return 2;
  }





// o = operation mode
// x = mux
// g = gain
// m = mode
//                oxxx gggm
// default 0x8583 1000 0101 1000 0011
//                1111 0101 1000 0011
//                1111 0101 1000 0011




  float max=getADS1115MaxGain(gain);

  mcp23x17_setPinOutputMode(lTrackForward, LOW);
  mcp23x17_setPinOutputMode(rTrackForward, LOW);
  mcp23x17_setPinOutputMode(lTrackReverse, LOW);
  mcp23x17_setPinOutputMode(rTrackReverse, LOW);
  

  if (doCalibration) {
    calibrate();
  }
  readCalibration();
  float lastVolts[4]={0,0,0,0};

  while (lastVolts[yThrottle]<0.1) {
    for (int i=0;i<throttleChannelCount;++i) {
      float v=readVoltageSingleShot(ADS1115_HANDLE, throttleChannels[i], gain);
      lastVolts[i]=v;
    }
  }


    printf("startVolts:   %12.6f %12.6f %12.6f %12.6f\n", 
             lastVolts[0], lastVolts[1], lastVolts[2], lastVolts[3]);

  float volts[4]={0,0,0,0};
  directionType direction=undetermined;
  directionType trackAction=goStraight;

  while (true) {
    long long now=currentTimeMillis();

    // bool retry=true;

    // while (retry) {
    //   retry=false;
    //   for (int i=0;i<throttleChannelCount;++i) {
    //     float v=readVoltageSingleShot(ADS1115_HANDLE, throttleChannels[i], gain);
    //     if (abs(v-lastVolts[i])/((lastVolts[i]+v)/2)>0.50) {
    //       printf("i=%d; lastVolts[i]=%6.4f; v=%6.4f\n",i,lastVolts[i],v);
    //       retry=true;
    //     }
    //     volts[i]=v;
    //   }
    // }

    for (int i=0;i<throttleChannelCount;++i) {
      float v=readVoltageSingleShot(ADS1115_HANDLE, throttleChannels[i], gain);
      volts[i]=v;
    }


    while (volts[yThrottle]<0.1) {
      for (int i=0;i<throttleChannelCount;++i) {
        if (volts[i]<=(-max) || volts[i]>=max) {
          printf("out of range\n");
          break;
        }
      }
    }


    printf("%lld %12.6f %12.6f %12.6f %12.6f\r", 
            now, volts[0], volts[1], volts[2], volts[3]);



    // printf("yMiddle      %6.4f\n",yMiddle);
    // printf("yResolution  %6.4f\n",yResolution);
    // printf("reverse tgt  %6.4f\n",yMiddle-yResolution);

    

    if (volts[yThrottle]>0.1) {
      if (volts[yThrottle]<yMiddle-yResolution) {         // move forward
        if (direction != forward) {
          direction=forward;
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
        if (direction != reverse) {
          direction = reverse;
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
        }
          allStop();
      }

      if (volts[xThrottle]>xMiddle+xResolution) {         // turn right
        if (trackAction!=turnRight) {
          trackAction=turnRight;
          const char *notes = "turn right";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrack, 0);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrack, 255);
        }
      } else if (volts[xThrottle]<xMiddle-xResolution) {  // turn left
        if (trackAction!=turnLeft) {
          trackAction=turnLeft;
          const char *notes = "turn left";
          printf("%lld %12.6f %12.6f %12.6f %12.6f; %s\n", 
            now, volts[0], volts[1], volts[2], volts[3], notes);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrack, 255);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrack, 0);

          printf("volts[x]: %6.4f \n",volts[xThrottle]);
          printf("xMiddle:  %6.4f %6.4f\n",xMiddle, xResolution);
          printf("xTarget:  %6.4f \n",xMiddle-xResolution);
          // exit(2);
        }
      } else {                                            // go straight
        if (trackAction!=goStraight) {
          trackAction=goStraight;
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + lTrack, 0);
          wiringPiI2CWriteReg8(pca9635Handle, 0x02 + rTrack, 0);
        }
      }
    }

    fflush(stdout);
  }
}

