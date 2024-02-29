/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * This library source code has been modified for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * Modified by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, June, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark/BNO080-STM32F4-SPI-LL-Driver
 */

/**
 * This library has been further modified for STM32F4 with the SPI and GPIO LL.
 * Additional features includes quarternion calculation incorporated in the lib.
 * Initialisation is now not the responsibility of the library but the user and
 * must be called before the init is run.
 * @author pipipipi2002 (Marvin Pranajaya)
 */

#ifndef	INC_BNO08X_LL_H
#define	INC_BNO08X_LL_H

#include "main.h"
#include <stdbool.h>

//Registers
enum Registers
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
};

#if defined (DISABLE_DEBUG)
	#define $INFO(fmt, ...)
	#define $ERROR(fmt, ...) 
	#define $SUCCESS(fmt, ...) 
#elif defined (USE_LOGGER)
	#define $INFO(fmt, ...) log_pInfo(fmt, ##__VA_ARGS__)
	#define $ERROR(fmt, ...) log_pError(fmt, ##__VA_ARGS__)
	#define $SUCCESS(fmt, ...) log_pSuccess(fmt, ##__VA_ARGS__)
#else
	#include <printf.h>
	#define $INFO(fmt, ...)     printf("[BNO08X] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $ERROR(fmt, ...)    printf("[BNO08X] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
	#define $SUCCESS(fmt, ...)  printf("[BNO08X] "); printf(fmt, ##__VA_ARGS__); printf("\n\r")
#endif 

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_UNCALIBRATED_GYRO 0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_RAW_ACCELEROMETER 0xE301
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_LINEAR_ACCELERATION 0xE303
#define FRS_RECORDID_GRAVITY 0xE304
#define FRS_RECORDID_RAW_GYROSCOPE 0xE305
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_GYROSCOPE_UNCALIBRATED 0xE307
#define FRS_RECORDID_RAW_MAGNETOMETER 0xE308
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_MAGNETIC_FIELD_UNCALIBRATED 0xE30A
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B
#define FRS_RECORDID_GAME_ROTATION_VECTOR 0xE30C
#define FRS_RECORDID_GEOMAGNETIC_ROTATION_VECTOR 0xE30D

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

bool BNO08X_init(SPI_TypeDef* spi, GPIO_TypeDef* intPort, uint16_t intPin, GPIO_TypeDef* rstPort, uint16_t rstPin, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* wakePort, uint16_t wakePin);
unsigned char SPI2_SendByte(unsigned char data);

int BNO08X_dataAvailable(void);
void BNO08X_parseCommandReport(void);
void BNO08X_parseInputReport(void);

void BNO08X_getRollPitchYaw(float* roll, float* pitch, float* yaw);
float BNO08X_getQuatI(void);
float BNO08X_getQuatJ(void);
float BNO08X_getQuatK(void);
float BNO08X_getQuatReal(void);
float BNO08X_getQuatRadianAccuracy(void);
uint8_t BNO08X_getQuatAccuracy(void);
float BNO08X_getAccelX(void);
float BNO08X_getAccelY(void);
float BNO08X_getAccelZ(void);
uint8_t BNO08X_getAccelAccuracy(void);
float BNO08X_getLinAccelX(void);
float BNO08X_getLinAccelY(void);
float BNO08X_getLinAccelZ(void);
uint8_t BNO08X_getLinAccelAccuracy(void);
float BNO08X_getGyroX(void);
float BNO08X_getGyroY(void);
float BNO08X_getGyroZ(void);
uint8_t BNO08X_getGyroAccuracy(void);
float BNO08X_getMagX(void);
float BNO08X_getMagY(void);
float BNO08X_getMagZ(void);
uint8_t BNO08X_getMagAccuracy(void);
uint16_t BNO08X_getStepCount(void);
uint8_t BNO08X_getStabilityClassifier(void);
uint8_t BNO08X_getActivityClassifier(void);
uint32_t BNO08X_getTimeStamp(void);
int16_t BNO08X_getQ1(uint16_t recordID);
int16_t BNO08X_getQ2(uint16_t recordID);
int16_t BNO08X_getQ3(uint16_t recordID);
float BNO08X_getResolution(uint16_t recordID);
float BNO08X_getRange(uint16_t recordID);

uint32_t BNO08X_readFRSword(uint16_t recordID, uint8_t wordNumber);
void BNO08X_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
int BNO08X_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);
void BNO08X_softReset(void);
uint8_t BNO08X_resetReason(void);

float BNO08X_qToFloat(int16_t fixedPointValue, uint8_t qPoint);

void BNO08X_enableRotationVector(uint16_t timeBetweenReports);
void BNO08X_enableGameRotationVector(uint16_t timeBetweenReports);
void BNO08X_enableAccelerometer(uint16_t timeBetweenReports);
void BNO08X_enableLinearAccelerometer(uint16_t timeBetweenReports);
void BNO08X_enableGyro(uint16_t timeBetweenReports);
void BNO08X_enableMagnetometer(uint16_t timeBetweenReports);
void BNO08X_enableStepCounter(uint16_t timeBetweenReports);
void BNO08X_enableStabilityClassifier(uint16_t timeBetweenReports);

void BNO08X_calibrateAccelerometer(void);
void BNO08X_calibrateGyro(void);
void BNO08X_calibrateMagnetometer(void);
void BNO08X_calibratePlanarAccelerometer(void);
void BNO08X_calibrateAll(void);
void BNO08X_endCalibration(void);
int BNO08X_calibrationComplete(void);

void BNO08X_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);
void BNO08X_sendCommand(uint8_t command);
void BNO08X_sendCalibrateCommand(uint8_t thingToCalibrate);
void BNO08X_requestCalibrationStatus(void);
void BNO08X_saveCalibration(void);

int BNO08X_waitForSPI(void);
int BNO08X_receivePacket(void);
int BNO08X_sendPacket(uint8_t channelNumber, uint8_t dataLength);

#endif // INC_BNO08X_LL_H
