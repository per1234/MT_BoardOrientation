/*
 * MT_BoardOrientation.cpp - Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * Currently using MPU6050 6-axis motion sensor.
 * Created by MTS Standish (mattThurstan), 2017.
 * Copyleft.
 */
 
/*
 * Code contained within this file 'MT_BoardOrientation.cpp' was initially built with reference code from the following sources:
 * 
 * 'MPU-6050 Accelerometer + Gyro' by "Krodal" (arduino.cc user ) - June 2012 - Open Source / Public Domain.
 * 
 * https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_raw/MPU6050_raw.ino
 * 
 * http://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/ 
 * - Done by Luis Ródenas <luisrodenaslorda@gmail.com>
 * - Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
 * 
 * https://vvvv.org/documentation/arduino02
 * 
============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "Arduino.h"
#include "MT_BoardOrientation.h"
#include <I2Cdev.h>                               //I2C devices
#include <MPU6050.h>                              //MPU6050 6-axis motion sensor
#include <Wire.h>

//#define DEBUG	//comment/un-comment

#ifdef DEBUG
  #define DEBUG_PRINT_HEADER(x); Serial.print(F("MT_BoardOrientation - "))
  #define DEBUG_PRINT(x);    Serial.print(x)
  #define DEBUG_PRINTF(x);    Serial.print(F(x))
  #define DEBUG_PRINTLN(x);  Serial.println(x)
  #define DEBUG_PRINTLNF(x); Serial.println(F(x))
#else
  #define DEBUG_PRINT_HEADER(x)
  #define DEBUG_PRINT(x)     //blank line
  #define DEBUG_PRINTF(x)    //blank line
  #define DEBUG_PRINTLN(x)   //blank line
  #define DEBUG_PRINTLNF(x)  //blank line
#endif

/*
inline unsigned long get_last_time();
inline unsigned long get_last_time();
inline float get_last_x_angle();
inline float get_last_y_angle();
inline float get_last_z_angle();
inline float get_last_gyro_x_angle();
inline float get_last_gyro_y_angle();
inline float get_last_gyro_z_angle();
*/

MT_BoardOrientation::MT_BoardOrientation() {
	
	/*----------------------------MPU6050 init---------------------------*/
	//X=Right/Left, Y=Forward/Backward, Z=Up/Down
	_mpu6050AccelOffset[0] = 436; 				//XYZ accel offsets to write to the MPU6050 - get from full calibration and save to memory
	_mpu6050AccelOffset[1] = 1956;
	_mpu6050AccelOffset[2] = 1318;
	_mpu6050GyroOffset[0] = 9;       			//XYZ gyro offsets to write to the MPU6050 - get from full calibration and save to memory
	_mpu6050GyroOffset[1] = -32; 
	_mpu6050GyroOffset[2] = 69;
	_mpu6050CalibratePrevMillis = 0;            //previous time for reference

	/*----------------------------stuff for filtering--------------------*/
	_mpu6050ReadPrevMillis = 0;         		//previous time for reference
	_mpu6050AccelZero[0] = 0;           		//XYZ quick calibration zero average save for accel - quick offsets to use whilst running
	_mpu6050AccelZero[1] = 0;
	_mpu6050AccelZero[2] = 0;
	_mpu6050GyroZero[0] = 0;            		//XYZ quick calibration zero average save for gyro
	_mpu6050GyroZero[1] = 0;
	_mpu6050GyroZero[2] = 0;
	_mpu6050AccelRead[0] = 0;                 	//XYZ Current accel reading
	_mpu6050AccelRead[1] = 0;
	_mpu6050AccelRead[2] = 0;
	_mpu6050GyroRead[0] = 0;                	//XYZ Current gyro reading
	_mpu6050GyroRead[1] = 0;
	_mpu6050GyroRead[2] = 0;
	_mpu6050AccelReadAverage[0] = 0;          	//XYZ averaged current accel reading. see calibration
	_mpu6050AccelReadAverage[1] = 0;
	_mpu6050AccelReadAverage[2] = 0;
	_mpu6050GyroReadAverage[0] = 0;           	//XYZ averaged current gyro reading. see calibration
	_mpu6050GyroReadAverage[1] = 0;
	_mpu6050GyroReadAverage[2] = 0;
	_mpu6050FilteredPrev[0] = 0;   				//XYZ previous filtered reading
	_mpu6050FilteredPrev[1] = 0;
	_mpu6050FilteredPrev[2] = 0;
	_mpu6050GyroPrev[0] = 0;                 	//XYZ last_gyro_x_angle
	_mpu6050GyroPrev[1] = 0;
	_mpu6050GyroPrev[2] = 0;
	_mpu6050Accel_yPrev = 0;                    //Y previous raw accleration y value

	/*----------------------------FINAL calculated numbers---------------*/
	_mpu6050FilteredCur[0] = 0;              	//XYZ FINAL filtered combined gyro/accel readings for use in calculating orientation
	_mpu6050FilteredCur[1] = 0;
	_mpu6050FilteredCur[2] = 0;
	
	/*----------------------------direction------------------------------*/
	_diAccelSave = 0;
	_diDirectionCounter = 0;            		//restricted by '_directionSampleTotal'
	_directionCur = 0;                 			// -1 = stationary, 0 = forward, 1=back, 2=up, 3=down, 4=left, 5=right

	/*----------------------------orientation----------------------------*/
	_orientation = 0;        					//0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
	orMatrix[0] = 0;             	    		//TEMP x =  0(low) / 1(mid) / 2(hi) - wanted to use -1, 0, 1 but too convoluted -- XYZ timed
	orMatrix[1] = 0;
	orMatrix[2] = 0;
	_orOrientationSave = 0;                		//used to hold the orientation during comparison
	_orOrientationTemp = 0;                  	//used to hold the orientation (then convert to _orientation)
	orFlag = false;                           	//flag 0 x
	orCounter = 0;                      		//TEMP time
	_orientationPrevMillis = 0;         		//previous time for reference
	_orientationTestSideMidpoint = 0;           //side LED strip midpoint, calculated in startup
	
	/*---vars and access function to help store the overall rotation angle of the sensor---*/
	last_read_time = 0UL;
	
	//TEST
	//unsigned long t = get_last_time(); //seems to be ok with only declaring it in the '.h' file. is this the correct way?
}

void MT_BoardOrientation::Init() {

	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      //join I2C bus (I2Cdev library doesn't do this automatically)
	  Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	  Fastwire::setup(400, true);
	#endif
	
	//initialise device
	DEBUG_PRINTLNF("Initializing MPU6050 on I2C...");
	_mpu6050.initialize();    //this gets stuck sometimes on the mini pro
	//statusLED.Blink2();

	//verify connection
	DEBUG_PRINTLNF("Testing MPU6050 connection...");
	DEBUG_PRINTLN(_mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	//set offsets 
	_mpu6050.setXAccelOffset(_mpu6050AccelOffset[0]);
	_mpu6050.setYAccelOffset(_mpu6050AccelOffset[1]);
	_mpu6050.setZAccelOffset(_mpu6050AccelOffset[2]);
	_mpu6050.setXGyroOffset(_mpu6050GyroOffset[0]);
	_mpu6050.setYGyroOffset(_mpu6050GyroOffset[1]);
	_mpu6050.setZGyroOffset(_mpu6050GyroOffset[2]); 
}

void MT_BoardOrientation::InitWithVars(int16_t accelOffest[3], int16_t gyroOffest[3]) {	
	_mpu6050AccelOffset[0] = accelOffest[0];
	_mpu6050AccelOffset[1] = accelOffest[1];
	_mpu6050AccelOffset[2] = accelOffest[2];
	_mpu6050GyroOffset[0] = gyroOffest[0];
	_mpu6050GyroOffset[1] = gyroOffest[1];
	_mpu6050GyroOffset[2] = gyroOffest[2];
	Init();
}

void MT_BoardOrientation::set_last_read_angle_data(unsigned long time, float accel_y, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
	last_read_time = time;
	_mpu6050Accel_yPrev = accel_y;
	_mpu6050FilteredPrev[0] = x;
	_mpu6050FilteredPrev[1] = y;
	_mpu6050FilteredPrev[2] = z;
	_mpu6050GyroPrev[0] = x_gyro;
	_mpu6050GyroPrev[1] = y_gyro;
	_mpu6050GyroPrev[2] = z_gyro;
}

void MT_BoardOrientation::ReadFiltered() {
	
	unsigned long mpu6050ReadCurMillis = millis();     //get current time

    //read raw values
    _mpu6050.getMotion6(&_mpu6050AccelRead[0], &_mpu6050AccelRead[1], &_mpu6050AccelRead[2], &_mpu6050GyroRead[0], &_mpu6050GyroRead[1], &_mpu6050GyroRead[2]);
    
    // Convert gyro values to degrees/sec
    float FS_SEL = 131;

    float gyro_x = ((float)_mpu6050GyroRead[0] - _mpu6050GyroZero[0])/FS_SEL;
    float gyro_y = ((float)_mpu6050GyroRead[1] - _mpu6050GyroZero[1])/FS_SEL;
    float gyro_z = ((float)_mpu6050GyroRead[2] - _mpu6050GyroZero[2])/FS_SEL;

    // Get raw acceleration values
    float accel_x = (float)_mpu6050AccelRead[0];
    float accel_y = (float)_mpu6050AccelRead[1];
    float accel_z = (float)_mpu6050AccelRead[2];

    //add to the average for direction calc
    //..this really needs to be a rolling average
    _diAccelSave += accel_y;
    _diDirectionCounter++;
    
    // Get angle values from accelerometer
    float RADIANS_TO_DEGREES = 180/3.14159;
    float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

    float accel_angle_z = 0;
    
    // Compute the (filtered) gyro angles
    float dt =(mpu6050ReadCurMillis - get_last_time())/1000.0;
    float gyro_angle_x = gyro_x*dt + get_last_x_angle();
    float gyro_angle_y = gyro_y*dt + get_last_y_angle();
    float gyro_angle_z = gyro_z*dt + get_last_z_angle();
    
    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
    
    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    float alpha = 0.96; //0.96;  //0.04sec ..erm
    float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
    float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
    float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

    //..this is annoying me ..so ..abracadabra, make minus, plus, stop annoying me..
    //
    //hohum, does effectively turn them from floats to ints, but one does not really require the accuracy
    if (angle_x > 180) { _mpu6050FilteredCur[0] = -abs(angle_x); }
    else if (angle_x < -180) { _mpu6050FilteredCur[0] = abs(angle_x); }
    else { _mpu6050FilteredCur[0] = angle_x; }  //oops, forgot these
    
    if (angle_y > 180) { _mpu6050FilteredCur[1] = -abs(angle_y); }
    else if (angle_y < -180) { _mpu6050FilteredCur[1] = abs(angle_y); }
    else { _mpu6050FilteredCur[1] = angle_y; }  //..
    
    if (angle_z > 180) { _mpu6050FilteredCur[2] = -abs(angle_z); }
    else if (angle_z < -180) { _mpu6050FilteredCur[2] = abs(angle_z); }
    else  {_mpu6050FilteredCur[2] = angle_z; }

    // Update the previous saved data with the latest values - saving un-adjusted values to previous, not minus/plus values..
    set_last_read_angle_data(mpu6050ReadCurMillis, accel_y, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

}

void MT_BoardOrientation::ReadDirection() {
	//dependant on whether tracking sub-mode is actually running, otherwise waste of processing..
	if (_diDirectionCounter >= _directionSampleTotal) {
	  //..this really needs to be a rolling average
	  unsigned int average = (_diAccelSave / _directionSampleTotal);
	  if (average > _mpu6050AccelZero[1] + 100) {
		_directionCur = 0;  //going forwards
	  } else if (average < _mpu6050AccelZero[1] - 100) {
		_directionCur = 1;  //going backwards
	  } else {
		//_directionCur = -1;  //stationary
	  }
	  _diAccelSave = 0;
	  _diDirectionCounter = 0;
	}
}

void MT_BoardOrientation::ReadOrientation() {

    float cutoff = 45;  //starting at zero calibration, we need to know 90deg either way, so 45 is halfway point anywhere from 0
    for (int i = 0; i < 3; i++) {
      if ( _mpu6050FilteredCur[i] < ( _mpu6050AccelZero[i] - cutoff ) ) { orMatrix[i] = 0; }
      else if ( _mpu6050FilteredCur[i] < ( _mpu6050AccelZero[i] + cutoff ) && _mpu6050FilteredCur[i] > ( _mpu6050AccelZero[i] - cutoff ) ) { orMatrix[i] = 1; }
      else if ( _mpu6050FilteredCur[i] > ( _mpu6050AccelZero[i] + cutoff ) ) { orMatrix[i] = 2; }
    }

    //compare 3-matrix, set orientation temp
    if (orMatrix[0] == 1 && orMatrix[1] == 1 ) {
            _orOrientationTemp = 0;
            if (orFlag == false) { _orOrientationSave = 0; orCounter = millis(); orFlag = true; }
          }  //1, 1, 1 - flat
//upside-down - have to work on this one..
//    else if (orMatrix[0] == 1
//          && (orMatrix[1] == 0 || orMatrix[1] == 2)
//          && orMatrix[2] == 0) {
//            _orOrientationTemp = 1;
//            if (orFlag == false) { _orOrientationSave = 1; orCounter = millis(); orFlag = true; }
//          }  //1, 0 or 2, 1 - upside-down - have to work on this one..
    else if (orMatrix[0] == 2 && orMatrix[1] == 1 ) { 
            _orOrientationTemp = 2;
            if (orFlag == false) { _orOrientationSave = 2; orCounter = millis(); orFlag = true; } 
          }  //2, 1, 1 - up
    else if (orMatrix[0] == 0 && orMatrix[1] == 1 ) { 
            _orOrientationTemp = 3;
            if (orFlag == false) { _orOrientationSave = 3; orCounter = millis(); orFlag = true; }
          }  //0, 1, 1 - down
    else if (orMatrix[0] == 1 && orMatrix[1] == 0 ) { 
            _orOrientationTemp = 4;
            if (orFlag == false) { _orOrientationSave = 4; orCounter = millis(); orFlag = true; }
          }  //1, 0, 1 - left  
    else if (orMatrix[0] == 1 && orMatrix[1] == 2 ) { 
            _orOrientationTemp = 5;
            if (orFlag == false) { _orOrientationSave = 5; orCounter = millis(); orFlag = true; }
          }  //1, 2, 1 - right

    unsigned long orGetMillis = millis();
    if (orFlag == true) {
      if ( (unsigned long) (orGetMillis - orCounter) >= orInterval) {
        if (_orOrientationSave == _orOrientationTemp) {
          //is the orientation still the same as when we took a sample and set the timer?
          //if so, set the actual orientation
          _orientation = _orOrientationTemp;
        }
        orFlag = false; //either way, reset
      }
    }

}




