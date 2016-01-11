#pragma once
#include <math.h>

class testahrs
{
public:
        double gravityx , gravityy, gravityz;
        double yaw ,yaw2,yawoffset;// from quadcopter's point of view
        double velocityx , velocityy , velocityz ;
        double tgravityz ;
        double accelerationx, accelerationy , accelerationz ;
        double accelerationxlp, accelerationylp , accelerationzlp ;
        double height ;
        double positionx , positiony ;
        double DEG2RAD;
        double RAD2DEG;
        double pitch , roll ,pitch2 ;
        double apitch , aroll , apitch2 ;
        double tval ;

        double prevbarometer ,barometerspeed ;
        long pbarometertime ;

        double magXmin ,magXmax;
        double magYmin, magYmax ;
        double magZmin , magZmax ;
        double magangle , magcorrection;

        double magcx , magcy , magcz , magnormal;
        double ogx , ogy , ogz ;

        double fmagcx = 0.0d, fmagcy = 0.0d, fmagcz = 0.0d;
        double magpitch = 0.0d, magroll = 0.0d, magyaw = 0.0d;
        double mgravityx = 0, mgravityy = 0, mgravityz = 0;
        double mgravityx2 = 0, mgravityy2 = 0, mgravityz2 = 1.0d;
        double tmagcorrectionx = 0.0d, tmagcorrectiony = 0.0d, tmagcorrectionz = 0.0d;

		testahrs(void);
        testahrs(double ax,double ay,double az);
	~testahrs(void);
	void _2drotate(double x, double y, double angle, double &resx, double &resy);
	double CalcAngle(double x, double y);
    void CalculateAcceleration(double ax, double ay, double az, double &ox, double &oy, double &oz);
    void updatebarometer(double h,long clock);
    void UpdateMagnetometer(double mgx,double mgy,double mgz);
	void update(double ax, double ay, double az, double gx, double gy, double gz, double dt);
};

