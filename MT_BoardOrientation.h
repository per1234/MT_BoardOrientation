/*
 * MT_BoardOrientation.h - Orientation setup library for long/short/skate boards. Used in longboardLight1 project.
 * Currently using MPU6050 6-axis motion sensor.
 * Created by MTS Standish (mattThurstan), 2017.
 * Copyleft.
 */
 
 /*
  * MPU6050: 			X=Right/Left, Y=Forward/Backward, Z=Up/Down
  * orientation (byte):	0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
  * direction (byte):	-1=stationary, 0=forward, 1=back, 2=up, 3=down, 4=left, 5=right
  */

#ifndef __MT_BOARDORIENTATION_H__
#define __MT_BOARDORIENTATION_H__

#include "Arduino.h"
#include <I2Cdev.h>                               	//I2C devices
#include <MPU6050.h>                              	//MPU6050 6-axis motion sensor
#include <Wire.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h - ???
  #include "Wire.h"
#endif

/*----------------------------main header declerations----------------------------*/
class MT_BoardOrientation
{
  private:
	/*----------------------------MPU6050 init---------------------------*/
	MPU6050 _mpu6050;  								//accel gyro;
	int16_t _mpu6050AccelOffset[3];       			//XYZ accel offsets to write to the MPU6050 - get from full calibration and save to memory
	int16_t _mpu6050GyroOffset[3];             		//XYZ gyro offsets to write to the MPU6050 - get from full calibration and save to memory
	
	/*----------------------------calibration----------------------------*/
	const int _mpu6050CalibrateSampleTotal = 100;	//how many samples to take at once when calibrating
	const int _mpu6050CalibrateAccelThreshold = 10;	//threshold tolerance for 'dead zone' at center of readings
	const int _mpu6050CalibrateGyroThreshold = 3; 	//..for gyro
	long _mpu6050CalibratePrevMillis;         		//previous time for reference
	const long _mpu6050CalibrateInterval = 1000;	//sampling interval in milliseconds
	const long _mpu6050CalibrateTimeout = 300000;	//sampling interval in milliseconds (5 mins)
	
	/*----------------------------stuff for filtering--------------------*/
	//const unsigned long _mpu6050ReadInterval = 20;	//read loop interval in milliseconds   //40 //1000
	unsigned long _mpu6050ReadPrevMillis;     		//previous time for reference
	float _mpu6050AccelZero[3];           			//XYZ quick calibration zero average save for accel - quick offsets to use whilst running
	float _mpu6050GyroZero[3];            			//XYZ quick calibration zero average save for gyro
	int16_t _mpu6050AccelRead[3];           		//XYZ Current accel reading
	int16_t _mpu6050GyroRead[3];            		//XYZ Current gyro reading
	int16_t _mpu6050AccelReadAverage[3];   			//XYZ averaged current accel reading. see calibration
	int16_t _mpu6050GyroReadAverage[3];     		//XYZ averaged current gyro reading. see calibration
	float _mpu6050FilteredPrev[3];  				//XYZ previous filtered reading. why have i not go this already ???
	float _mpu6050GyroPrev[3];             			//XYZ last_gyro_x_angle;
	float _mpu6050Accel_yPrev;                		//Y previous raw accleration y value

	/*----------------------------FINAL calculated numbers---------------*/
	float _mpu6050FilteredCur[3];             		//XYZ FINAL filtered combined gyro/accel readings for use in calculating orientation

	/*----------------------------direction------------------------------*/
	//prob won't use 'stationary' cos the calculations will need something to get started, otherwise they will be a frame behind. better to have wrong direction for a split second, than have more complicated code
	//also might try this as the average of a rolling buffer cos for 10 samples we wait 200 or 400ms..
	const byte _directionSampleTotal = 10;     		//how many times to sample direction before making a decision on whether it is true or not
	unsigned int _diAccelSave;
	byte _diDirectionCounter;                 		//restricted by '_directionSampleTotal'
	byte _directionCur;                				// -1 = stationary, 0 = forward, 1=back, 2=up, 3=down, 4=left, 5=right

	/*----------------------------orientation----------------------------*/
	byte _orientation;                  			//0=flat, 1=upside-down, 2=up, 3=down, 4=left-side, 5=right-side
	byte orMatrix[3];           					//TEMP x =  0(low) / 1(mid) / 2(hi)       - wanted to use -1, 0, 1 but too convoluted    -- XYZ timed
	byte _orOrientationSave;             			//used to hold the orientation during comparison
	byte _orOrientationTemp;           				//used to hold the orientation (then convert to _orientation)
	boolean orFlag;                     			//flag 0 x
	unsigned long orCounter;            			//TEMP time
	//const unsigned long _orientationInterval = 100;	//main orientation read loop interval in milliseconds
	const unsigned long orInterval = 450;			//interval at which to check whether flags have changed - are we still in the same orientation - how long to trigger
	unsigned long _orientationPrevMillis;  			//previous time for reference
	byte _orientationTestSideMidpoint;     			//side LED strip midpoint, calculated in startup

	/*---vars and access function to help store the overall rotation angle of the sensor---*/
	unsigned long last_read_time;

	inline unsigned long get_last_time() {return last_read_time;}
	inline float get_last_x_angle() {return _mpu6050FilteredPrev[0];}
	inline float get_last_y_angle() {return _mpu6050FilteredPrev[1];}
	inline float get_last_z_angle() {return _mpu6050FilteredPrev[2];}
	inline float get_last_gyro_x_angle() {return _mpu6050GyroPrev[0];}
	inline float get_last_gyro_y_angle() {return _mpu6050GyroPrev[1];}
	inline float get_last_gyro_z_angle() {return _mpu6050GyroPrev[2];}
	void set_last_read_angle_data(unsigned long time, float accel_y, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro);
	
	void doMPU6050ReadAverage();

  public:
	MT_BoardOrientation();
	void Init();
	void InitWithVars(int16_t accelOffest[3], int16_t gyroOffest[3]);
	void ReadFiltered();							//called from main program loop, every 1 sec
	void ReadDirection();							//called from main program loop every N ms (if tracking sub-mode is running)
	void ReadAverage();								//for calibration
	void ReadOrientation();							//called from main program loop every N ms
	
	byte GetDirection() { return _directionCur; }
	byte GetOrientation() { return _orientation; }
	
	void QuickCalibration();
	void FullCalibration();
	
	int16_t GetMPU6050AccelOffsetX() { return _mpu6050AccelOffset[0]; }
	int16_t GetMPU6050AccelOffsetY() { return _mpu6050AccelOffset[1]; }
	int16_t GetMPU6050AccelOffsetZ() { return _mpu6050AccelOffset[2]; }
	
	int16_t GetMPU6050GyroOffsetX() { return _mpu6050GyroOffset[0]; }
	int16_t GetMPU6050GyroOffsetY() { return _mpu6050GyroOffset[1]; }
	int16_t GetMPU6050GyroOffsetZ() { return _mpu6050GyroOffset[2]; }
	
	void SetMPU6050AccelOffset(int16_t ao[3]);
	void SetMPU6050GyroOffset(int16_t go[3]);
	
};

#endif


