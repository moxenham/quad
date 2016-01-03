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
        const double DEG2RAD = 3.14159265354 / 180.0;
        const double RAD2DEG = 180.0 / 3.14159265354;
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

