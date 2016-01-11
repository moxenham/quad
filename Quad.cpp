#include <cmath>
#include <sys/time.h>
#include <termios.h>
//#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <fstream>
#include <sys/stat.h>

#include "Navio/Ublox.h" 
#include "Navio/PCA9685.h"
#include "Navio/MPU9250.h"
#include "Navio/MS5611.h"
#include "mmapGpio.h"

#define	HANDLE	int
#define	INVALID_HANDLE_VALUE	0
#define	DWORD	long

//=============================================================================
double frequency = 0.0f;
PCA9685 pwm;
const float RADIANS_TO_DEGREES = 180.0f/3.14159265354;
const float RAD2DEG = RADIANS_TO_DEGREES;
const float DEG_TO_RAD = 3.14159265354/180.0f;
const float DEG2RAD = DEG_TO_RAD;
struct timeval tv;
bool quiting = false;
const float ESCCentre = 1460;

bool armed = false;
int rollchannel = 0;
int pitchchannel = 1;
int throttlechannel = 2;
int yawchannel = 3;

float fb = 0.0f, lr = 0.0f;// fb stands for front/back and lr is yup you guessed it left/right
	
long ptime = 0;

float fspeedtime0 = 0.0f;
float fspeedtime1 = 0.0f;
float fspeedtime2 = 0.0f;
float fspeedtime3 = 0.0f;

float afb2 = 0.0f,alr2 = 0.0f;
float afb = 0.0f,alr = 0.0f,arr = 0.0f,thr = 0.0f,athr = 0.0f,rr = 0.0f,lza = 0.0f;
float afbav = 0.0f,alrav = 0.0f;
float afb2av = 0.0f,alr2av = 0.0f,yawcentre = 0.0f;
float prevafb2 = 0.0f,prevalr2 = 0.0f;
float prev2afb2 = 0.0f,prev2alr2 = 0.0f;

float kp = 1.0,ki = 0.0001,kd = 0.00;// const float kp = 1.0,ki = 0.009,kd = -0.05;, kd=-0.90
float kp2 = 0.090,ki2 = 0.0300,kd2 = 0.300,kd22 = 0.00;// .250 and .003 float kp2 = 0.250,ki2 = 0.000,kd2 = 0.003;kd2=0.0045, -0.008
float yawkp = -0.6,yawkd = 0.30,yawki = 0.000;//float yawkp = -2.0,yawkd = 1.0,yawki = 0.001;
float hoverscale = 43.0f/50.0f;

float sdfb = 0.0f,sdpfb = 0.0f;
float sdlr = 0.0f,sdplr = 0.0f;
float yawav = 0.0f;
float yaw = 0.0f;

float cgafb = 0.0f,cgalr = 0.0f,cgarr = 0.0f;// centre of gravity adjustments

float magangle = 0.0f;
float magcorrection = 0.0f;

// virtual horizon values
float vhzxz = -3.00,vhzyz = 0.0;// xz=roll, yz=pitch (might also need a xy, yaw equivelent)

MS5611 baro;
unsigned long long lastbarometerupdate = 0;
int barometerstate = 0;
float barometerpressure = 0.0f;
float tbaro = 0.0f;
float barometerheight = 0.0f;
float barometerheightlong = 0.0f;
float barometeri = 0.0f;
float barometerheightbase = 0.0f;// start height above sea level (set to 0 so we get an offset from the start position)
float barometertempbase = 0.0f;
float barometerpressurebase = 0.0f;
float barometerspeed = 0.0f;
int barometerfirst = 0;
unsigned long long lastpressureread = 0;

struct parms
{
	int datalength;
};

struct parms savedparameters;
float *calibdata = NULL;

void LoadParms()
{
	HANDLE fl2;
	fl2 = open("parameters.quad",O_RDONLY,S_IREAD);
	if (fl2!=INVALID_HANDLE_VALUE)
	{
		//SetFilePointer(fl2,0,NULL,FILE_END);
		DWORD wr = read(fl2,&savedparameters,sizeof(parms));
		if (savedparameters.datalength>0)
		{
			calibdata = new float[savedparameters.datalength];
			wr = read(fl2,calibdata,sizeof(float)*savedparameters.datalength);
		}
	}
	close(fl2);

}

void SaveParms()
{
	HANDLE fl2;
	fl2 = open("parameters.quad",O_CREAT | O_WRONLY,S_IWRITE);
	if (fl2!=INVALID_HANDLE_VALUE)
	{
		//SetFilePointer(fl2,0,NULL,FILE_END);
		DWORD wr = write(fl2,&savedparameters,sizeof(parms));
		if (savedparameters.datalength>0)
		{
			wr = write(fl2,calibdata,sizeof(float)*savedparameters.datalength);
		}
	}
	close(fl2);
}

void ParmsAddValue(float l,float ax,float ay,float az)
{
	// should check for nearby values first, if we are too near a values that is already in the list dont add (or update)
	// but for now just make sure we can add a value
	int dl = savedparameters.datalength/4;
	float *calibdata2 = new float[dl+4];
	memcpy(calibdata,calibdata2,savedparameters.datalength);
	calibdata2[dl] = l;
	calibdata2[dl+1] = ax;
	calibdata2[dl+2] = ay;
	calibdata2[dl+3] = az;
	delete(calibdata);
	calibdata = calibdata2;
	savedparameters.datalength+=4;
	SaveParms();
}

void SetPWM(float ms1,float ms2,float ms3,float ms4)
{
	int ims1 = round((ms1 * 4096.f) / (1000.f / frequency) - 1);
	int ims2 = round((ms2 * 4096.f) / (1000.f / frequency) - 1);
	int ims3 = round((ms3 * 4096.f) / (1000.f / frequency) - 1);
	int ims4 = round((ms4 * 4096.f) / (1000.f / frequency) - 1);

	if (ims1>4095) ims1 = 4095;
	if (ims2>4095) ims2 = 4095;
	if (ims3>4095) ims3 = 4095;
	if (ims4>4095) ims4 = 4095;
	if (ims1<0) ims1 = 0;
	if (ims2<0) ims2 = 0;
	if (ims3<0) ims3 = 0;
	if (ims4<0) ims4 = 0;

	uint8_t data[16] = {0,0,ims1&0xFF,ims1>>8,
						0,0,ims2&0xFF,ims2>>8,
						0,0,ims3&0xFF,ims3>>8,
						0,0,ims4&0xFF,ims4>>8
						};
    I2Cdev::writeBytes(PCA9685_DEFAULT_ADDRESS, PCA9685_RA_LED0_ON_L + 4 * 3, 16, data);
	
}

void SetLED(int red,int green,int blue)
{
	// values are inverted, 4095=minimum illumination
	uint8_t data[12] = {0,0,blue&0xFF,blue>>8,
						0,0,green&0xFF,green>>8,
						0,0,red&0xFF,red>>8,
						};
    I2Cdev::writeBytes(PCA9685_DEFAULT_ADDRESS, PCA9685_RA_LED0_ON_L + 4 * 0, 12, data);


}

float GetNormal(float x,float y,float z)
{
	return sqrt(x*x+y*y+z*z);
}

float constrain(float val,float min,float max)
{
	float res = val;
	if (res<min)
		res = min;
	if (res>max)
		res = max;
	return res;
}

void _2drotate(float v1,float v2,float &v1o,float &v2o,float a)
{
	v1o = cos(a*DEG2RAD)*v1-sin(a*DEG2RAD)*v2;
	v2o = sin(a*DEG2RAD)*v1+cos(a*DEG2RAD)*v2;
}

float CalcAngle(float x, float y)
{
	float rat = x / y;

	float a = (float)atan(rat) * RAD2DEG;
	if (y < 0) a = a + 180;
	if (a > 180) a = a - 360;
	if (a < -180) a = a + 360;
	return a;
}


float channels[8];
float channelscopy[8];
float tchannels[8];
float channelsmin[8];
float channelsmax[8];
float channelsmid[8];
unsigned int currentChannel = 0;
unsigned int previousTick;
unsigned int deltaTime;
unsigned long long radiotime = 0;
pthread_mutex_t radiomutex;
pthread_t nradiothread;

void *radiothread(void *args)
{
	mmapGpio gpio;
	gpio.setPinDir(16,mmapGpio::INPUT); 
	gpio.setPinDir(20,mmapGpio::INPUT); 
	gpio.setPinDir(26,mmapGpio::INPUT); 
	gpio.setPinDir(21,mmapGpio::OUTPUT); 

	
	unsigned int timediff = 0;
	unsigned int time = 0;
	bool laststate = false;
	gpio.writePinLow(21);
	bool reading = false;
	unsigned char tmpbuffer[12];
	int inputbyte = 0;
	int bit =1;
	int counter = 0;
	unsigned long long ptime=0;
	unsigned int lastclock = 0;
	unsigned int lastclockvalue = 0;
	struct timeval tv2;
	int startbits = 1;
	int startbitcount = 0;
	while(1)
	{
		// read the pin and cycle the cycle
		unsigned int clockvalue = gpio.readPin(20);
		bool ardstate = false;
		if (clockvalue!=0)
			ardstate = true;

		if (ardstate==laststate)
		{
			lastclockvalue = clockvalue;
			unsigned int pinvalue = gpio.readPin(16);
			unsigned int pinvalue2 = gpio.readPin(26);
			if (laststate==true)
			{
				gpio.writePinLow(21);
				laststate = false;
			}
			else
			{
				gpio.writePinHigh(21);
				laststate = true;
			}
			if (reading==true)
			{// read 12 bytes
				if (pinvalue!=0)
				{// HIGH
					tmpbuffer[inputbyte] = tmpbuffer[inputbyte] | bit;
				}
				else
				{// LOW
					tmpbuffer[inputbyte] = tmpbuffer[inputbyte] & ~bit;
				}
				bit = bit<<1;
				if (bit>128)
				{
					bit = 1;
					inputbyte++;
				}
				if (pinvalue2!=0)
				{// HIGH
					tmpbuffer[inputbyte] = tmpbuffer[inputbyte] | bit;
				}
				else
				{// LOW
					tmpbuffer[inputbyte] = tmpbuffer[inputbyte] & ~bit;
				}
				bit = bit<<1;
				if (bit>128)
				{
					bit = 1;
					inputbyte++;
				}
				
				if (inputbyte>11)
				{
					counter++;
					reading = false;
					tchannels[0] = tmpbuffer[0]*256+tmpbuffer[1];
					tchannels[1] = tmpbuffer[2]*256+tmpbuffer[3];
					tchannels[2] = tmpbuffer[4]*256+tmpbuffer[5];
					tchannels[3] = tmpbuffer[6]*256+tmpbuffer[7];
					tchannels[4] = tmpbuffer[8]*256+tmpbuffer[9];
					tchannels[5] = tmpbuffer[10]*256+tmpbuffer[11];
					// check all channels are valid data, if one is invalid then drop this packet
					if ((tchannels[0]>=0) && (tchannels[0]<2500))
					{
						if ((tchannels[1]>=0) && (tchannels[1]<2500))
						{
							if ((tchannels[2]>=0) && (tchannels[2]<2500))
							{
								if ((tchannels[3]>=0) && (tchannels[3]<2500))
								{
									if ((tchannels[4]>=0) && (tchannels[4]<2500))
									{
										if ((tchannels[5]>=0) && (tchannels[5]<2500))
										{
											gettimeofday(&tv2,NULL);
											radiotime = tv2.tv_sec;
											radiotime = radiotime*1000000 + tv2.tv_usec;
											pthread_mutex_lock(&radiomutex);
											channelscopy[0] = channelscopy[0]*0.8+tchannels[0]*0.2;
											channelscopy[1] = channelscopy[1]*0.8+tchannels[1]*0.2;
											channelscopy[2] = channelscopy[2]*0.8+tchannels[2]*0.2;
											channelscopy[3] = channelscopy[3]*0.8+tchannels[3]*0.2;
											channelscopy[4] = channelscopy[4]*0.7+tchannels[4]*0.3;
											channelscopy[5] = channelscopy[5]*0.7+tchannels[5]*0.3;
											pthread_mutex_unlock(&radiomutex);
										}
									}
								}
							}
						}
					}
				}
			}
			else
			{// detect start bit(s)
				if ((pinvalue!=0) && (pinvalue2==0))
				{
					startbitcount++;
					if (startbitcount>=startbits)
					{
						reading = true;
						inputbyte = 0;
						bit = 1;
						startbitcount = 0;
					}
				}
				else
				{
					startbitcount = 0;
				}
			}
		}
		if (!reading)
			usleep(1000);
	}
}

void InitRadioThread()
{
	int ret = pthread_mutex_init(&radiomutex,NULL);
	ret = pthread_create(&nradiothread,NULL,radiothread,NULL);
}

void FinishRadioThread()
{
	quiting = true;
	usleep(2000);
	pthread_mutex_destroy(&radiomutex);
}

int pwmupdate = 0;
bool upsidedown = false;
int upsidedownstate = 0;
float greafb = 0.0f;
double gscale = 0.0f;
long jittermin = 1000000;
long jittermax = -1000000;
double jittermean = 0.0f;
long jittercount = 0;

float SubtractAngle(float a,float b)
{ // a-b but modular between -180 +180, target angle is b, source angle is a
	float res = a-b;
	if (res<-180)
	{
		//res = 180-res;
		res+=360;
		//res = -res;
	}
	if (res>180)
	{
		//res = 180-res;
		res-=360;
		//res = -res;
	}
	return res;
}

void UpdatePidGyros(float tacx,float tacy,float tacz,float gyx,float gyy,float gyz)
{
	float acx = tacx;
	float acy = tacy;
	float acz = tacz;
	if (upsidedown)
	{// we're upsidedown - should probably have some leeway here
		acz = -acz;
		acy = -acy; // see what this does, might have to adjust other values based on it being inverted
		acx = -acx;
		fb = -fb;
	}
    float lza = yaw+magcorrection;
	if (lza>180)
		lza-=360;
	if (lza<-180)
		lza+=360;
		
  unsigned long long ctime=0;
  gettimeofday(&tv,NULL);
  ctime = tv.tv_sec;
	ctime = ctime*1000000 + tv.tv_usec;

  //gscale = (ctime-ptime);
  long jitter = (ctime-ptime);

  if (jitter<jittermin)
    jittermin = jitter;
  if (jitter>jittermax)
    jittermax = jitter;
  jittermean += jitter;
  jittercount++;

  
  gscale = ((double)(jitter))/1000000.0f;
  
  double gscale1 = DEG_TO_RAD*gscale;
  ptime = ctime;
  double cax = acx;
  double cay = acy;
  double caz = acz;
  
// Stability PID
	// Normalise Gravity
	double acnormal = cax*cax+cay*cay+caz*caz;
	acnormal = sqrt(acnormal);
	double cax2 = cax;
	cax2 = cax2/acnormal;
	double cay2 = cay;
	cay2 = cay2/acnormal;
	double caz2 = caz;
	caz2 = caz2/acnormal;
  
	float afberr = 0.0f;
	float alrerr = 0.0f;
	double tafb = (cay2-fb);
	afberr = constrain(kp*tafb*250,-300,300);
    afb = afberr;
	double talr = (cax2-lr);
	alrerr = constrain(kp*talr*-250,-300,300);
    alr = alrerr;

	
	if (thr>10)
	{
		afbav=constrain(afbav+constrain(afb*gscale,-5,5),-50/ki,50/ki);
		alrav=constrain(alrav+constrain(alr*gscale,-5,5),-50/ki,50/ki);
	}
	afb+=constrain(afbav*ki,-50,50);
	alr+=constrain(alrav*ki,-50,50);
	
	sdfb = (afberr-sdpfb);
	sdlr = (alrerr-sdplr);

	afb+=kd*sdfb*kp;
	alr+=kd*sdlr*kp;
	
	sdpfb = afberr;
	sdplr = alrerr;
	
	//afb = 0.0f;
	//alr = 0.0f;
	
// Rotation (Gyro) PID
        float fgyx = gyx;
        float fgyy = gyy;
        float fgyz = gyz;
        fgyx = fgyx;
        fgyy = fgyy;
        fgyz = fgyz;

		if (thr>10)
		{
			yawav+=SubtractAngle(lza,yawcentre)*gscale;
		}
        //arr = yawkp*rr+yawkd*fgyz+constrain(yawkp*(SubtractAngle(lza,yawcentre)),-20,20)+constrain(yawav*yawki,-5,5);// add some p to control it
        arr = yawkp*rr+yawkd*fgyz+yawkp*(SubtractAngle(lza,yawcentre))+yawav*yawki;// add some p to control it
        
        float reafb = afb+fgyx;//gyx//rafb
        float realr = alr+fgyy;//gyy//ralr
		greafb = reafb;
        if (thr>10)
        {
          afb2av+=reafb*gscale;
          alr2av+=realr*gscale;
          afb2av = constrain(afb2av,-10/ki2,10/ki2);
          alr2av = constrain(alr2av,-10/ki2,10/ki2);// 10/ki2 sets the maximum output for the I term to 10
        }
        else if (thr<10)
        {
          afb2av = 0;
          alr2av = 0;
          yawcentre = lza;
        }
        
        float rafb2 = (fgyx-prevafb2); // use fgyx instead of reafb to remove the control back kick
        float ralr2 = (fgyy-prevalr2); // use fgyy instead of realr to remove the control back kick
        prevafb2 = fgyx;
        prevalr2 = fgyy;
        
        // second PID
        afb2 = kp2*reafb+ki2*afb2av+kd2*rafb2*kp2;
        alr2 = kp2*realr+ki2*alr2av+kd2*ralr2*kp2;
        
		float t = afb2-prev2afb2;
		afb2+=kd22*t;
		t = alr2-prev2alr2;
		alr2+=kd22*t;
		
		
        afb2 = constrain(afb2,-20,20)*hoverscale;
        alr2 = constrain(alr2,-20,20)*hoverscale;
        arr = constrain(arr,-10,10)*hoverscale;
		
		prev2afb2 = afb2/hoverscale;
		prev2alr2 = alr2/hoverscale;

        athr = (((thr*1.1)-50)/5);
        athr = athr*athr*athr/20+50;
		athr = athr*hoverscale; // recenter does mean 100% is only 60% of max power, but probably a good thing. Adjust 30 to better hover speed when found
		
//        float ang = abs(last_x_angle)+abs(last_y_angle);
        float adj = 1;
//        if ((ang>0) && (ang<45))
//		{
          //adj = constrain((90/(90-ang))*sqrt(2)/2,1.0,1.5);
//		  adj = constrain(1/cos(ang*DEG2RAD),0.0f,1.5f);
//		}
        
		if (thr>=10)
		{
			//waaccelz = waaccelz*0.95f+worldlinearaccelf.z*0.05f;
			//athr = athr+constrain(waaccelz*0.20f,-5,5);//-constrain(worldlinearvelocity.z*0.1,-5,5); CHECK SIGN!
			
		}
		
        athr = athr*adj;
		
        athr = constrain(athr,-100,100);

        afb2+=cgafb;
        alr2+=cgalr;
        arr+=cgarr;
        
		if (upsidedown)
			athr = -athr;
		
        float thrscale = 1.0f;
        fspeedtime0 = constrain(athr+afb2*thrscale+alr2*thrscale+arr*thrscale,-100,100); // front left
        fspeedtime1 = constrain(athr+afb2*thrscale-alr2*thrscale-arr*thrscale,-100,100); // front right
        fspeedtime2 = constrain(athr-afb2*thrscale-alr2*thrscale+arr*thrscale,-100,100); // back right
        fspeedtime3 = constrain(athr-afb2*thrscale+alr2*thrscale-arr*thrscale,-100,100); // back left
        
		fspeedtime0 = fspeedtime0*4+ESCCentre;
		fspeedtime1 = fspeedtime1*4+ESCCentre;
		fspeedtime2 = fspeedtime2*4+ESCCentre;
		fspeedtime3 = fspeedtime3*4+ESCCentre;

        if (thr<10)
        { // if throttle less than 10% then zero the motors
          fspeedtime0 = ESCCentre;
          fspeedtime1 = ESCCentre;
          fspeedtime2 = ESCCentre;
          fspeedtime3 = ESCCentre;
        }
          
		  // should now set the pwm speeds
		pwmupdate++;

		if (armed)
		{
			SetPWM(fspeedtime0/1000.0f,fspeedtime1/1000.0f,fspeedtime2/1000.0f,fspeedtime3/1000.0f);
			pwmupdate = 0;
		}
}

void ResetPWM()
{// set minimum values
		pwm.setPWMmS(3,ESCCentre/1000.0f);
		pwm.setPWMmS(4,ESCCentre/1000.0f);
		pwm.setPWMmS(5,ESCCentre/1000.0f);
		pwm.setPWMmS(6,ESCCentre/1000.0f);
}

void CalibratePPM()
{
	for (int r=0;r<8;r++)
	{
		if (r!=throttlechannel)
		{
			if (channels[r]<channelsmin[r])
				channelsmin[r] = channels[r];
			if (channels[r]>channelsmax[r])
				channelsmax[r] = channels[r];
		}
	}
}

void SetMid()
{
	for (int r=0;r<4;r++)
	{
		channelsmid[r] = channels[r];
	}
}

int main()
{
	//-------------------------------------------------------------------------
	channelscopy[0] = 1000;
	channelscopy[1] = 1000;
	channelscopy[2] = 1000;
	channelscopy[3] = 1000;
	channelscopy[4] = 1000;
	channelscopy[5] = 1000;
	InitRadioThread();
	for (int r=0;r<8;r++)
	{
		channelsmin[r] = 1000;
		channelsmax[r] = 1900;
		channelsmid[r] = 1000;
		channels[r] = 1000;// defaults
	}
	channelsmin[throttlechannel] = 1015;
	channelsmax[throttlechannel] = 2000;
	channelsmid[4] = 1400;
	channelsmid[5] = 1400;
	pthread_mutex_lock(&radiomutex);
	channels[0] = channelscopy[0];
	channels[1] = channelscopy[1];
	channels[2] = channelscopy[2];
	channels[3] = channelscopy[3];
	channels[4] = channelscopy[4];
	channels[5] = channelscopy[5];
	channels[6] = channelscopy[6];
	channels[7] = channelscopy[7];
	pthread_mutex_unlock(&radiomutex);
		
	while( ((channels[0]==1000) || (channels[1]==1000) || (channels[2]==1000) || (channels[3]==1000)))// || (i<500))
	{
		usleep(5000);
		pthread_mutex_lock(&radiomutex);
		channels[0] = channelscopy[0];
		channels[1] = channelscopy[1];
		channels[2] = channelscopy[2];
		channels[3] = channelscopy[3];
		channels[4] = channelscopy[4];
		channels[5] = channelscopy[5];
		channels[6] = channelscopy[6];
		channels[7] = channelscopy[7];
		pthread_mutex_unlock(&radiomutex);

	} // wait for initial transmitter values
	for (int r=0;r<50;r++)
	{
		usleep(5000);
		pthread_mutex_lock(&radiomutex);
		channels[0] = channelscopy[0];
		channels[1] = channelscopy[1];
		channels[2] = channelscopy[2];
		channels[3] = channelscopy[3];
		channels[4] = channelscopy[4];
		channels[5] = channelscopy[5];
		channels[6] = channelscopy[6];
		channels[7] = channelscopy[7];
		pthread_mutex_unlock(&radiomutex);
	}
	SetMid();
	printf("%4.0f,%4.0f, %4.0f, %4.0f\n",channels[0],channels[1],channels[2],channels[3]);
	

    pwm.initialize();
    pwm.setFrequency(50);
	frequency = pwm.getFrequency();

    baro.initialize();

	MPU9250 imu;
	imu.initialize();
	imu.set_acc_scale(BITS_FS_4G);

	float tax, tay, taz, tgx, tgy, tgz, tmx, tmy, tmz;

    //-------------------------------------------------------------------------
	float ax = 0.0f,ay = 0.0f,az = 1.0f;
	unsigned long long lastdisplayupdate = 0;
	unsigned long long lastimuread = 0;
	
	gettimeofday(&tv,NULL);
	lastdisplayupdate = tv.tv_sec;
	lastdisplayupdate = lastdisplayupdate*1000000 + tv.tv_usec;
	lastimuread = lastdisplayupdate;
		
	float gyxav = 0.0f,gyyav = 0.0f,gyzav = 0.0f;
	float axav = 0.0f,ayav = 0.0f,azav = 0.0f;
	for (int r=0;r<1000;r++)
	{
        imu.getMotion9(&tax, &tay, &taz, &tgx, &tgy, &tgz, &tmx, &tmy, &tmz);
		axav+=tax;
		ayav+=tay;
		azav+=taz;

		gyxav+=tgx;
		gyyav+=tgy;
		gyzav+=tgz;
		usleep(2000);
	}
	axav = axav/1000.0f;
	ayav = ayav/1000.0f;
	azav = azav/1000.0f;
	// calculate an average accelerometer stationary length
	float accelerometeravlength = sqrt(axav*axav+ayav*ayav+azav*azav);

	gyxav = gyxav/1000.0f;
	gyyav = gyyav/1000.0f;
	gyzav = gyzav/1000.0f;

	bool ledstate = false;
	std::ofstream logfile;
    logfile.open("log.txt", std::ios_base::app);
	float avtax,avtay,avtaz,avtgx,avtgy,avtgz;
	float tax2,tay2,taz2,tgx2,tgy2,tgz2;
	int imureadcount = 0;
	int pidupdatecount = 0;
	unsigned long long ctime=0;
	long sleeptime = 1500;
	int targetfrequency = 1000;// must be a multiplier of the below targetcount divisor
	int targetcount = targetfrequency/50; // 50 is the frequency of pid updates, must be an integer divisor of the above targetfrequency
	double avlen = 0.0d;
	double vax = 0.0d,vay = 0.0d,vaz = 0.0d;
    while(!quiting) {
		avtax = 0.f;
		avtay = 0.f;
		avtaz = 0.f;
		avtgx = 0.f;
		avtgy = 0.f;
		avtgz = 0.f;
		for (int l=0;l<targetcount;l++)
		{
			usleep(sleeptime);
			imu.getMotion9(&tax2, &tay2, &taz2, &tgx2, &tgy2, &tgz2, &tmx, &tmy, &tmz);
			imureadcount++;
			tgx2-=gyxav;
			tgy2-=gyyav;
			tgz2-=gyzav;
			
			avtax+=tax2;
			avtay+=tay2;
			avtaz+=taz2;
			avtgx+=tgx2;
			avtgy+=tgy2;
			avtgz+=tgz2;
		}
		tax = avtax/targetcount;
		tay = avtay/targetcount;
		taz = avtaz/targetcount;
		double mt = sqrt(tax*tax+tay*tay+taz*taz);
		avlen+=mt;
		/*tax = tax/accelerometeravlength;
		tay = tay/accelerometeravlength;
		taz = taz/accelerometeravlength;*/

		tgx = avtgx/targetcount;
		tgy = avtgy/targetcount;
		tgz = avtgz/targetcount;
		
		gettimeofday(&tv,NULL);
		ctime = tv.tv_sec;
		ctime = ctime*1000000 + tv.tv_usec;
		float timescale = (ctime-lastimuread)/1000000.0f;
		lastimuread = ctime;
		
		float rax,ray,raz;
		_2drotate(ax,az,rax,raz,tgy*timescale);
		_2drotate(ay,raz,ray,az,-tgx*timescale);
		ax = rax;ay = ray;
		_2drotate(ax,ay,rax,ray,-tgz*timescale);
		ax = rax;ay = ray;
		
		
		// use the acceleromter to correct for drift on the pitch and roll axis
		ax = ax*0.98f+tax*0.02f;
		ay = ay*0.98f+tay*0.02f;
		az = az*0.98f+taz*0.02f;
		// calc a simple pitch/roll
		float roll = CalcAngle(ax,az);
		// remove roll so the pitch calculation is correct
		_2drotate(ax,az,rax,raz,-roll);
		float pitch = CalcAngle(ay,raz);

		// to calculate current realworld acceleration, remove yaw, then recaulculate pitch and roll, apply the same calculation to gravity and subtract it (gravity should end up 0,0,1)
		float taxwg,taywg,tazwg;
		float rwax,rway,rwaz;
		_2drotate(tax,taz,taxwg,tazwg,roll);
		_2drotate(tay,tazwg,rway,rwaz,pitch);
		rwax = taxwg;
		
		yaw-=tgz*timescale;
		
		
		pthread_mutex_lock(&radiomutex);
		channels[0] = channelscopy[0];
		channels[1] = channelscopy[1];
		channels[2] = channelscopy[2];
		channels[3] = channelscopy[3];
		channels[4] = channelscopy[4];
		channels[5] = channelscopy[5];
		channels[6] = channelscopy[6];
		channels[7] = channelscopy[7];
		pthread_mutex_unlock(&radiomutex);
		if (channels[4]>channelsmid[4])
		{
			armed = true;
		}
		else
		{
			if (armed)
			{
				armed = false;
				ResetPWM();
			}
		}
		if (channels[5]>channelsmid[5])
		{
			if (!upsidedown)
			{
				upsidedown = false;//true
			}
		}
		else
		{
			if (upsidedown)
			{
				upsidedown = false;
			}
		}
		
		CalibratePPM();

		thr = (channels[throttlechannel]-channelsmin[throttlechannel])*100/(channelsmax[throttlechannel]-channelsmin[throttlechannel]);
		fb = (channels[pitchchannel]-channelsmid[pitchchannel]);
		if (fb<0)
		{
			fb = (fb/(channelsmid[pitchchannel]-channelsmin[pitchchannel]))/3.0f;
		}
		else
		{
			fb = (fb/(channelsmax[pitchchannel]-channelsmid[pitchchannel]))/3.0f;
		}
		float t = (channels[pitchchannel]-channelsmid[pitchchannel]);
		if ((t<100) && (t>-100))
			fb = 0.0f;
		lr = (channels[rollchannel]-channelsmid[rollchannel]);
		if (lr<0)
		{
			lr = (lr/(channelsmid[rollchannel]-channelsmin[rollchannel]))/3.0f;
		}
		else
		{
			lr = (lr/(channelsmax[rollchannel]-channelsmid[rollchannel]))/3.0f;
		}
		t = (channels[rollchannel]-channelsmid[rollchannel]);
		if ((t<100) && (t>-100))
			lr = 0.0f;

		if (thr<10)
			thr = 0;

		float tvx = 0,tvy = 0,tvz = 0,tvz2 = 0;
		// rotate gravity by the virtual horizon values
		_2drotate(ax,az,tvx,tvz,vhzxz);
		_2drotate(ay,tvz,tvy,tvz2,vhzyz);
		//double vax = tvx,vay = tvy,vaz = tvz2;
		vax +=(tvx-vax)*0.5;// simple smoothing filter
		vay +=(tvy-vay)*0.5;
		vaz +=(tvz2-vaz)*0.5;
		
		if (ctime-lastbarometerupdate>=21000)
		{
			if (barometerstate==0)
			{
				baro.refreshPressure();
				barometerstate = 1;
			}
			else if (barometerstate==1)
			{
				baro.readPressure();
				baro.refreshTemperature();
				barometerstate = 2;
			}
			else if (barometerstate==2)
			{
				baro.readTemperature();
				baro.calculatePressureAndTemperature();
				//printf("Temperature(C): %f Pressure(millibar): %f\n", baro.getTemperature(), baro.getPressure());
				// do whatever with the data
				//barometerpressure = (barometerpressure*19+my_filter->do_sample( baro.getPressure() ))/20;
				tbaro = baro.getPressure();
				if ((tbaro<1200) && (tbaro>600))
				{
					//barometerpressure = my_filter->do_sample( tbaro );
					barometerpressure = tbaro;
					//barometerpressure = (barometerpressure*19+baro.getPressure())/20;
					if (barometerfirst<=10)
					{
						barometerpressure = tbaro;
						barometertempbase = baro.getTemperature();
						barometerpressurebase = barometerpressure;
					}
					if (barometerfirst<500)
					{
						barometertempbase = (barometertempbase*19+baro.getTemperature())/20;
						barometerpressurebase = (barometerpressurebase*19+barometerpressure)/20;// *6894.75729f for Pa instead of PSI
						barometerfirst++;
						barometerheightlong = barometerheight;
					}
					float previousbarometerheight = barometerheight;
					//barometerheight = previousbarometerheight*0.9f+(barometerheightbase+((barometertempbase+273.15)/-0.0065)*( pow((barometerpressure/barometerpressurebase),((-8.31432f*-0.0065f)/(9.80665f*0.0289644f))) -1.0f))*0.1f;
					barometerheight = (barometerheightbase+((barometertempbase+273.15)/-0.0065)*( pow((barometerpressure/barometerpressurebase),((-8.31432f*-0.0065f)/(9.80665f*0.0289644f))) -1.0f));
					barometerheightlong = barometerheightlong*0.9975+barometerheight*0.0025;
					if (barometerfirst>=500)
						barometerspeed = (barometerheight-previousbarometerheight)/((ctime-lastpressureread)/1000000.0f);// take into account the time taken to read
				}
				lastpressureread = ctime;
				//barometerstate = 0;
				baro.refreshPressure();
				barometerstate = 1;
			}
			lastbarometerupdate = ctime;
		}

	
		UpdatePidGyros(vax,vay,vaz,tgx,tgy,tgz);
		
		logfile << ctime << "," << tax << "," << tay << "," << taz << "," << tgx << "," << tgy << "," << tgz << "," << ax  << "," << ay  << "," << az << "," << yaw << "," << afb2 << "," << alr2 << "," << arr << "," << afbav << "," << alrav << "," << afb2av << "," << alr2av << "," << barometerpressure << "," << tmx << "," << tmy << "," << tmz << std::endl;
		//logfile << 0 ctime << "," << 1 tax << "," << 2 tay << "," << 3 taz << "," << 4 tgx << "," << 5 tgy << "," << 6 tgz << "," << 7 ax  << "," << 8 ay  << "," << 9 az << "," << 10 yaw << "," << 11 afb2 << "," << 12 alr2 << "," << 13 arr << "," << 14 afbav << "," << 15 alrav << "," << 16 afb2av << "," << 17 alr2av << "," << 18 barometerheight << "," << 19 tmx << "," << 20 tmy << "," << 21 tmz << std::endl;
		
		pidupdatecount++;
		if ((ctime-lastdisplayupdate)>1000000)
		{
			lastdisplayupdate = ctime;
						if (ledstate)
			{
				SetLED(0,4095,4095);// red on, green and blue off
				ledstate = false;
			}
			else
			{
				SetLED(4095,4095,4095);// all off
				ledstate = true;
			}
			
			long ti = imureadcount-targetfrequency;// auto adjust the sleep so it hits the target frequency
			sleeptime+=ti;
			
			avlen = avlen/pidupdatecount;
			double tax2 = (tax/avlen)-tax;
			double tay2 = (tay/avlen)-tay;
			double taz2 = (taz/avlen)-taz;
			printf("%i %i %4.3f ,%4.3f, %4.3f avlen: %2.4f %4.3f ,%4.3f, %4.3f \n",imureadcount,pidupdatecount,vax,vay,vaz,avlen,tax2,tay2,taz2);
			imureadcount = 0;
			pidupdatecount = 0;
			avlen = 0.0d;
			//printf("%4.0f,%4.0f, %4.0f, %4.0f\n",channels[0],channels[1],channels[2],channels[3]);
			//printf("%4.2f, %4.2f, %4.2f\n",fb,lr,thr);
		}
    }
}
