#include "stdafx.h"
#include "testahrs.h"


testahrs::testahrs(void)
{
        gravityx = 0.0; gravityy = 0.0; gravityz = 1.0;
        yaw = 0.0f;yaw2 = 0.0;yawoffset = 0.0;// from quadcopter's point of view
        velocityx = 0.0; velocityy = 0.0; velocityz = 0.0;
        tgravityz = 0.0;
        accelerationx = 0.0; accelerationy = 0.0; accelerationz = 0.0;
        accelerationxlp = 0.0; accelerationylp = 0.0; accelerationzlp = 0.0;
        height = 0.0;
        positionx = 0.0; positiony = 0.0;
        pitch = 0.0; roll = 0.0;pitch2 = 0.0;
        apitch = 0.0; aroll = 0.0; apitch2 = 0.0;
        tval = 0.0;

        prevbarometer = 0.0;barometerspeed = 0.0;
        pbarometertime = 0;

        magXmin = 10000.0;magXmax = -10000.0;
        magYmin = 10000.0; magYmax = -10000.0;
        magZmin = 10000.0; magZmax = -10000.0;
        magangle = 0.0; magcorrection = 0.0;

        magcx = 0.0; magcy = 0.0; magcz = 0.0;
        ogx = 0.0; ogy = 0.0; ogz = 0.0;

        DEG2RAD = 3.14159265354 / 180.0;
        RAD2DEG = 180.0 / 3.14159265354;

        fmagcx = 0.0; fmagcy = 0.0; fmagcz = 0.0;
        magpitch = 0.0; magroll = 0.0; magyaw = 0.0;
        mgravityx = 0; mgravityy = 0; mgravityz = 0;
        mgravityx2 = 0; mgravityy2 = 0; mgravityz2 = 1.0;
        tmagcorrectionx = 0.0; tmagcorrectiony = 0.0, tmagcorrectionz = 0.0;
}


testahrs::~testahrs(void)
{
}


        testahrs::testahrs(double ax,double ay,double az)
        {// initialise the values with the raw ones
            gravityx = ax;
            gravityy = ay;
            gravityz = az;
        }

        void testahrs::_2drotate(double x, double y, double angle, double &resx, double &resy)
        {
            resx = cos(angle * DEG2RAD) * x - sin(angle * DEG2RAD) * y;
            resy = sin(angle * DEG2RAD) * x + cos(angle * DEG2RAD) * y;
        }

        double testahrs::CalcAngle(double x, double y)
        {
            return atan2(y, x) * RAD2DEG;
        }

        void testahrs::updatebarometer(double h,long clock)
        {
            if (pbarometertime>0)
                barometerspeed = (h - prevbarometer) * ((clock - pbarometertime) / 1000000.0); // completely unreliable, it amplifies the noise from the barometer
            pbarometertime = clock;
            prevbarometer = h;
        }


        void testahrs::CalculateAcceleration(double ax, double ay, double az, double &ox, double &oy, double &oz)
        {
            double l = ax * ax + ay * ay + az * az;
            double tx = ax / l, ty = ay / l, tz = az / l;
            double tgrx, tgry, tgrz;
            double tgravityx = tx, tgravityy = ty;
            tgravityz = tz;
            double tpitch, troll, tpitch2;
            if ((tgravityz < 0.5) && (tgravityz > -0.5) && (tgravityy > -0.5) && (tgravityy < 0.5))
            {// pitch becomes unreliable as the roll approaches 90 degrees. pitch2 is used to compensate for this by being calculated after the roll is removed 
                //(when the quadcopter is on the side the y/z plane is aligned with gravity, so you cant calculate pitch for the same reason yaw cant be calculated when upright, but once roll is removed from gravity the "new" pitch2 is no longer aligned with gravity)
                // so why not swap the order of pitch and roll you ask? because you run into the same problem but when pitch heads towards 90 degrees the roll axis becomes aligned with gravity.
                tpitch = 0.0;
            }
            else
            {
                tpitch = CalcAngle(tgravityz, tgravityy);
            }
            // remove pitch then calc roll
            _2drotate(tgravityy, tgravityz, tpitch, tgry, tgrz); tgravityy = tgry; tgravityz = tgrz;
            troll = CalcAngle(tgravityz, tgravityx);
            // remove the roll
            _2drotate(tgravityx, tgravityz, troll, tgrx, tgrz); tgravityx = tgrx; tgravityz = tgrz;
            tpitch2 = CalcAngle(tgravityz, tgravityy);

            double tax = ax-tx, tay = ay-ty, taz = az-tz;
            _2drotate(tay, taz, pitch, tgry, tgrz); tay = tgry; taz = tgrz;
            _2drotate(tax, taz, roll, tgrx, tgrz); tax = tgrx; taz = tgrz;
            _2drotate(tay, taz, pitch2, tgry, tgrz); tay = tgry; taz = tgrz;
            _2drotate(tax, tay, yaw, tgrx, tgry); tax = tgrx; tay = tgry;

            ox = tax; oy = tay; oz = taz;
        }

        void testahrs::update(double ax, double ay, double az, double pgx, double pgy, double pgz, double dt)
        {
            // apply a small low pass filter to the gyro's
            double gx = (pgx + ogx) / 2.0;
            double gy = (pgy + ogy) / 2.0;
            double gz = (pgz + ogz) / 2.0;
            gx = pgx; gy = pgy; gz = pgz;
            ogx = pgx;
            ogy = pgy;
            ogz = pgz;

            double tgrx, tgry, tgrz, tgrx2, tgry2, tgrz2;

            // rotate gravity by the gyro's value to estimate the new position
            _2drotate(gravityy, gravityz, gx*dt, tgry, tgrz);
            _2drotate(gravityx, tgrz, gy*dt, tgrx, tgrz2);
            _2drotate(tgrx, tgry, gz*dt, tgrx2, tgry2);
            // tgrx2,tgry2,tgrz2 are the new values
            gravityx += (tgrx2 - gravityx) * 1.0;// 0.99 is how much we trust the gyro rotated value
            gravityy += (tgry2 - gravityy) * 1.0;
            gravityz += (tgrz2 - gravityz) * 1.0;

            // rotate the magnetometer gravity estimation by the gyro's
            _2drotate(mgravityy2, mgravityz2, gx * dt, tgry, tgrz);
            _2drotate(mgravityx2, tgrz, gy * dt, tgrx, tgrz2);
            _2drotate(tgrx, tgry, gz * dt, tgrx2, tgry2);
            mgravityx2 = tgrx2; mgravityy2 = tgry2; mgravityz2 = tgrz2;
            
            //gravityx += (mgravityx2 - gravityx) * 0.02;// 0.02 is how much we trust the magnetometer
            //gravityy += (mgravityy2 - gravityy) * 0.02;
            //gravityz += (mgravityz2 - gravityz) * 0.02;

            double lnm = sqrt(ax * ax + ay * ay + az * az);
            // also add the accelerometer readings (normalised), but we trust that less because its very noisy
            gravityx += ((ax / lnm) - gravityx) * 0.02;
            gravityy += ((ay / lnm) - gravityy) * 0.02;
            gravityz += ((az / lnm) - gravityz) * 0.02;

            // normalise gravity
            double scl = sqrt(gravityx * gravityx + gravityy * gravityy + gravityz * gravityz);
            gravityx /= scl;
            gravityy /= scl;
            gravityz /= scl;
            double tgravityx = gravityx,tgravityy = gravityy;
            tgravityz = gravityz;
            if ((tgravityz < 0.25) && (tgravityz > -0.5) && (tgravityy > -0.5) && (tgravityy < 0.5))
            {// pitch becomes unreliable as the roll approaches 90 degrees. pitch2 is used to compensate for this by being calculated after the roll is removed 
                //(when the quadcopter is on the side the y/z plane is aligned with gravity, so you cant calculate pitch for the same reason yaw cant be calculated when upright, but once roll is removed from gravity the "new" pitch2 is no longer aligned with gravity)
                // so why not swap the order of pitch and roll you ask? because you run into the same problem but when pitch heads towards 90 degrees the roll axis becomes aligned with gravity.
                pitch = 0.0;
                yawoffset = 0.0;
            }
            else
            {
                pitch = CalcAngle(tgravityz, tgravityy);
                yawoffset = 0.0;
            }
            // remove pitch then calc roll
            _2drotate(tgravityy, tgravityz, pitch, tgry, tgrz);tgravityy = tgry;tgravityz = tgrz;
            roll = CalcAngle(tgravityz, tgravityx);
            // remove the roll
            _2drotate(tgravityx, tgravityz, roll, tgrx, tgrz);tgravityx = tgrx;tgravityz = tgrz;

            pitch2 = CalcAngle(tgravityz, tgravityy);

            //
            tgravityx = gravityx; tgravityy = gravityy; tgravityz = gravityz;// just to check the gravity goes to 0,0,1 when rotated
            _2drotate(tgravityy, tgravityz, pitch, tgry, tgrz); tgravityy = tgry; tgravityz = tgrz;
            _2drotate(tgravityx, tgravityz, roll, tgrx, tgrz); tgravityx = tgrx; tgravityz = tgrz;
            _2drotate(tgravityy, tgravityz, pitch2, tgry, tgrz); tgravityy = tgry; tgravityz = tgrz;

            // rotate the gyro values by pitch and roll to calculate the real world yaw rotation speed
            _2drotate(gy, gz, -pitch, tgry, tgrz);
            _2drotate(gx, tgrz, roll, tgrx, tgrz2);
            _2drotate(tgry, tgrz2, pitch2, tgry2, tgrz);
            //_2drotate(tgrx, tgry, yaw, out tgrx2, out tgry2);
            yaw += dt * tgrz;
            if (yaw >= 180.0) yaw -= 360.0;
            if (yaw <= -180.0) yaw += 360.0;
            // apply a filter using the magnetometer correction
            /*double ma = magangle - yaw;
            if (ma > 180)
                ma -= 360;
            if (ma < -180)
                ma += 360;
            //yaw += (magangle - yaw) * 0.05d;
            yaw += (ma) * 0.005d;
            if (yaw >= 180.0d) yaw -= 360.0d;
            if (yaw <= -180.0d) yaw += 360.0d;*/

            // rotate the accelerometer reading by pitch/roll and yaw to find a real world acceleration
            double tax = ax, tay = ay, taz = az;
            _2drotate(tay, taz, pitch, tgry, tgrz); tay = tgry; taz = tgrz;
            _2drotate(tax, taz, roll, tgrx, tgrz); tax = tgrx; taz = tgrz;
            _2drotate(tay, taz, pitch2, tgry, tgrz); tay = tgry; taz = tgrz;
            _2drotate(tax, tay, yaw, tgrx, tgry); tax = tgrx; tay = tgry;
            tgrx2 = tax;
            tgry2 = tay;
            tgrz2 = taz;

            // and subtract gravity from z
            tgrz2 -= 1.0;// tgravityz;

            //CalculateAcceleration(ax, ay, az, out tgrx2, out tgry2, out tgrz2);

            accelerationx = tgrx2;
            //if ((accelerationx < 0.03) && (accelerationx > -0.03)) accelerationx = 0.0d;
            accelerationy = tgry2;
            //if ((accelerationy < 0.03) && (accelerationy > -0.03)) accelerationy = 0.0d;
            accelerationz = tgrz2;
            //if ((accelerationz < 0.03) && (accelerationz > -0.03)) accelerationz = 0.0d;

            //accelerationxlp = accelerationxlp * 0.95 + accelerationx * 0.05d;// low pass filter on accelerometers to create a high pass filter on velocity
            //accelerationylp = accelerationylp * 0.95 + accelerationy * 0.05d;
            //accelerationzlp = accelerationzlp * 0.999 + accelerationz * 0.001d;

            velocityx += (accelerationx - accelerationxlp) * dt*9.80665;// remember to multiply by the scale factor (gravity)
            velocityy += (accelerationy - accelerationylp) * dt * 9.80665;
            velocityz += (accelerationz - accelerationzlp) * dt * 9.80665;
            //velocityx *= 0.99d;// slowely cycle velocities down to account for inaccuracies
            //velocityy *= 0.99d;
            //velocityz *= 0.99d;
            //velocityz += (barometerspeed - velocityz) * 0.02d; // barometerspeed is too noisy, dont use

            positionx += velocityx * dt;
            positiony += velocityy * dt;

            height += velocityz * dt;

            //height += (prevbarometer - height) * 0.0005d;// barometer is a bit noisy so only use it to compensate for drift from the accelerometer
        }

        void testahrs::UpdateMagnetometer(double mgx,double mgy,double mgz)
        {
            double* magnetometer_data = new double[3];
			            // first check min/max values (always running calibration, will probably need to be reset to defaults if the quad is moved a significant distance)
            magnetometer_data[0] = mgx;
            magnetometer_data[1] = mgy;
            magnetometer_data[2] = mgz;
            if (magnetometer_data[0] < magXmin) magXmin = magnetometer_data[0];
            if (magnetometer_data[0] > magXmax) magXmax = magnetometer_data[0];

            if (magnetometer_data[1] < magYmin) magYmin = magnetometer_data[1];
            if (magnetometer_data[1] > magYmax) magYmax = magnetometer_data[1];

            if (magnetometer_data[2] < magZmin) magZmin = magnetometer_data[2];
            if (magnetometer_data[2] > magZmax) magZmax = magnetometer_data[2];
            if ((magXmax - magXmin) > 1)
            {
                magcx = magnetometer_data[0];
                magcx = ((magcx - (magXmax + magXmin) / 2)) / ((magXmax - magXmin) / 2);
            }
            if ((magYmax - magYmin) > 1)
            {
                magcy = magnetometer_data[1];
                magcy = ((magcy - (magYmax + magYmin) / 2) / ((magYmax - magYmin) / 2));
            }
            if ((magZmax - magZmin) > 1)
            {
                magcz = magnetometer_data[2];
                magcz = ((magcz - (magZmax + magZmin) / 2) / ((magZmax - magZmin) / 2)); // was magYmax-magYmin)/2
            }
            // normalise
            magnormal = magcx * magcx + magcy * magcy + magcz * magcz;
            magnormal = sqrt(magnormal);
            if (magnormal <= 0.01f)
                magnormal = 0.01f;
            magcx = magcx / magnormal;
            magcy = magcy / magnormal;
            magcz = magcz / magnormal;

            double tmx, tmy, tmz;
            // flip magcx and magcy, seems the magnetometer axis are different than the accelerometer
            tmx = magcy; magcy = magcx; magcx = tmx;

            double omagcx = magcx, omagcy = magcy, omagcz = magcz;

            _2drotate(magcy, magcz, -pitch, tmy, tmz); magcy = tmy; magcz = tmz; // not sure why these are - probably something to do with the direction of the magnetometer axis'
            _2drotate(magcx, magcz, -roll, tmx, tmz); magcx = tmx; magcz = tmz;
            _2drotate(magcy, magcz, -pitch2, tmy, tmz); magcy = tmy; magcz = tmz;

            magangle = CalcAngle(magcx, magcy);

            // this should point south in the northern hemisphere and north in the southern hemisphere

            // rotate magcx,magcy by the calculated angle, the results should end up always pointing in roughly the same direction (once calibration is done). Just here for validation.
            _2drotate(magcx, magcy, -magangle, tmx, tmy);
            magcx = tmx;
            magcy = tmy;


            /// experimental magnetometer correction for pitch and roll
            // take an average of the mag world reading (with respect to gravity being straight down)
            fmagcx += (magcx-fmagcx) * 0.01;
            fmagcy += (magcy-fmagcy) * 0.01;
            fmagcz += (magcz-fmagcz) * 0.01;

            // calculate pitch and roll of current mag reading
            magpitch = CalcAngle(omagcz, omagcy);
            // remove pitch then calc roll
            double tx,ty, tz;
            _2drotate(omagcy, omagcz, magpitch, ty, tz);
            magroll = CalcAngle(tz, omagcx);

            mgravityx = gravityx; mgravityy = gravityy; mgravityz = gravityz;
            // now calculate gravity with respect to the magnetometer axis
            _2drotate(mgravityy, mgravityz, -magpitch, ty, tz); mgravityy = ty; mgravityz = tz;
            _2drotate(mgravityx, mgravityz, -magroll, tx, tz); mgravityx = tx; mgravityz = tz;

            // calculate the yaw of the gravity reading and remove it
            magyaw = CalcAngle(mgravityx, mgravityy);
            _2drotate(mgravityx, mgravityy, -magyaw, tx, ty); mgravityx = tx; mgravityy = ty;

            // this vector should 

            // calculate pitch and roll of world mag reading
            double ofmagcx = fmagcx, ofmagcy = fmagcy, ofmagcz = fmagcz;
            double magpitch2 = CalcAngle(ofmagcz, ofmagcy);
            // remove pitch then calc roll
            _2drotate(ofmagcy, ofmagcz, magpitch2, ty, tz);
            double magroll2 = CalcAngle(tz, ofmagcx);

            mgravityx2 = 0; mgravityy2 = 0; mgravityz2 = 1.0;
            // now calculate gravity with respect to the magnetometer axis
            _2drotate(mgravityy2, mgravityz2, magpitch2, ty, tz); mgravityy2 = ty; mgravityz2 = tz;
            _2drotate(mgravityx2, mgravityz2, magroll2, tx, tz); mgravityx2 = tx; mgravityz2 = tz;

            // calculate the yaw of the gravity reading and remove it (shouldnt be one, but just to check)
            double magyaw2 = CalcAngle(mgravityx2, mgravityy2);
            _2drotate(mgravityx2, mgravityy2, magyaw2, tx, ty); mgravityx2 = tx; mgravityy2 = ty;

            // adjust mgravity towards mgravity2
            tmagcorrectionx= (mgravityx2 - mgravityx) * 0.1;// just so I can graph the changes
            tmagcorrectiony = (mgravityy2 - mgravityy) * 0.1;
            tmagcorrectionz = (mgravityz2 - mgravityz) * 0.1;

            mgravityx += (mgravityx2 - mgravityx) * 0.1;// magnetometer can be noisey so dont trust it so much
            mgravityy += (mgravityy2 - mgravityy) * 0.1;
            mgravityz += (mgravityz2 - mgravityz) * 0.1;

            // rotate mgravity back to the normal reading
            _2drotate(mgravityx, mgravityy, magyaw, tx, ty); mgravityx = tx; mgravityy = ty;
            _2drotate(mgravityx, mgravityz, magroll, tx, tz); mgravityx = tx; mgravityz = tz;
            _2drotate(mgravityy, mgravityz, magpitch, ty, tz); mgravityy = ty; mgravityz = tz;

            _2drotate(mgravityx2, mgravityy2, magyaw, tx, ty); mgravityx2 = tx; mgravityy2 = ty;
            _2drotate(mgravityx2, mgravityz2, magroll, tx, tz); mgravityx2 = tx; mgravityz2 = tz;
            _2drotate(mgravityy2, mgravityz2, magpitch, ty, tz); mgravityy2 = ty; mgravityz2 = tz;

            // this should be the new gravity reading
            //gravityx = mgravityx; gravityy = mgravityy; gravityz = mgravityz;
			delete(magnetometer_data);
        }

