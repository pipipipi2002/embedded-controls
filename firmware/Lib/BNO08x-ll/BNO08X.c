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

#include "BNO08X.h"
#include <math.h>


//Global Variables
uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

//These are the raw sensor values pulled from the user requested Input Report
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences; //Array that store the confidences of the 9 possible activities
uint8_t calibrationStatus;	 //Byte R0 of ME Calibration Response

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
//See the read metadata example for more info
int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;

static GPIO_TypeDef *_intPort, *_resetPort, *_csPort, *_wakePort;
static uint16_t _intPin, _resetPin, _csPin, _wakePin;
static SPI_TypeDef* _spi = NULL;

/**
 * @brief initialisation function. User need to init SPI and GPIO and other related
 * peripherals before using this library
 * 
 * @param spi 
 * @param intPort 
 * @param intPin 
 * @param rstPort 
 * @param rstPin 
 * @param csPort 
 * @param csPin 
 * @param wakePort 
 * @param wakePin 
 * @return true 
 * @return false 
 */
bool BNO08X_init(SPI_TypeDef* spi, GPIO_TypeDef* intPort, uint16_t intPin, GPIO_TypeDef* rstPort, uint16_t rstPin, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* wakePort, uint16_t wakePin)
{
    _spi = spi;

    _intPort = intPort;
    _intPin = intPin;
    _resetPort = rstPort;
    _resetPin = rstPin;
    _csPort = csPort;
    _csPin = csPin;
    _wakePort = wakePort;
    _wakePin = wakePin;

    $INFO("Start Initialisation.");

    // HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET); // Deselect CS
    LL_GPIO_SetOutputPin(_csPort, _csPin);
    // HAL_GPIO_WritePin(_wakePort, _wakePin, GPIO_PIN_SET); // PS0 set to high for SPI comms
    LL_GPIO_SetOutputPin(_wakePort, _wakePin);
    // HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_RESET); // reset active low
    LL_GPIO_ResetOutputPin(_resetPort, _resetPin);
    HAL_Delay(200);
    // HAL_GPIO_WritePin(_resetPort, _resetPin, GPIO_PIN_SET); // reset active low
    LL_GPIO_SetOutputPin(_resetPort, _resetPin);

    /* Wait for advertisement message */
    BNO08X_waitForSPI();
    BNO08X_receivePacket();
    
    /* Wait for unsolicited initialize response */
    BNO08X_waitForSPI();
    BNO08X_receivePacket();

    //Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;   // Request the product ID and reset info
	shtpData[1] = 0;						        // Reserved
	
	//Transmit packet on channel 2, 2 bytes
	BNO08X_sendPacket(CHANNEL_CONTROL, 2);
	
	//Now we wait for response
	BNO08X_waitForSPI();
	if (BNO08X_receivePacket() == 1)
	{
		$INFO("header: %d %d %d %d\n", shtpHeader[0], shtpHeader[1], shtpHeader[2], shtpHeader[3]);
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			$SUCCESS("BNO08X who_am_i = 0x%02x...ok\n\n", shtpData[0]);
			return true;
		}// Sensor OK
	}
	
	$ERROR("BNO08X Not OK: 0x%02x Should be 0x%02x\n", shtpData[0], SHTP_REPORT_PRODUCT_ID_RESPONSE);
	return false; // Something went wrong
}

unsigned char SPI2_SendByte(unsigned char data)
{
    while(LL_SPI_IsActiveFlag_TXE(_spi)==RESET) {};
	LL_SPI_TransmitData8(_spi, data);
	
	while(LL_SPI_IsActiveFlag_RXNE(_spi)==RESET) {};
	return LL_SPI_ReceiveData8(_spi);
}


//////////////////////////////////////////////////////////////////////////
//init
//////////////////////////////////////////////////////////////////////////

//Updates the latest variables if possible
//Returns false if new readings are not available
int BNO08X_dataAvailable(void)
{
	//If we have an interrupt pin connection available, check if data is available.
	//If int pin is NULL, then we'll rely on BNO080_receivePacket() to timeout
	//See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
	// if (HAL_GPIO_ReadPin(_intPort, _intPin) == 1 )
	// 	return (0);

	if (BNO08X_receivePacket() == 1)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			BNO08X_parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
			return (1);
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			BNO08X_parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
			return (1);
		}
	}
	return (0);
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
void BNO08X_parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO08X responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO08X_parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | (shtpData[3] << (8 * 2)) | (shtpData[2] << (8 * 1)) | (shtpData[1] << (8 * 0));

	uint8_t status = shtpData[7] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[10] << 8 | shtpData[9];
	uint16_t data2 = (uint16_t)shtpData[12] << 8 | shtpData[11];
	uint16_t data3 = (uint16_t)shtpData[14] << 8 | shtpData[13];
	uint16_t data4 = 0;
	uint16_t data5 = 0;

	if (dataLength > 14)
	{
		data4 = (uint16_t)shtpData[16] << 8 | shtpData[15];
	}
	if (dataLength > 16)
	{
		data5 = (uint16_t)shtpData[18] << 8 | shtpData[17];
	}

	//Store these generic values to their proper global variable
	switch(shtpData[5])
	{
		case SENSOR_REPORTID_ACCELEROMETER:
		{
			accelAccuracy = status;
			rawAccelX = data1;
			rawAccelY = data2;
			rawAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_LINEAR_ACCELERATION:
		{
			accelLinAccuracy = status;
			rawLinAccelX = data1;
			rawLinAccelY = data2;
			rawLinAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_GYROSCOPE:
		{
			gyroAccuracy = status;
			rawGyroX = data1;
			rawGyroY = data2;
			rawGyroZ = data3;
			break;
		}
		case SENSOR_REPORTID_MAGNETIC_FIELD:
		{
			magAccuracy = status;
			rawMagX = data1;
			rawMagY = data2;
			rawMagZ = data3;
			break;
		}
		case SENSOR_REPORTID_ROTATION_VECTOR:
		case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
		{
			quatAccuracy = status;
			rawQuatI = data1;
			rawQuatJ = data2;
			rawQuatK = data3;
			rawQuatReal = data4;
			rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
			break;
		}
		case SENSOR_REPORTID_STEP_COUNTER:
		{
			stepCount = data3; //Bytes 8/9
			break;
		}
		case SENSOR_REPORTID_STABILITY_CLASSIFIER:
		{
			stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
			break;
		}
		case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
		{
			activityClassifier = shtpData[5 + 5]; //Most likely state

			//Load activity classification confidences into the array
			for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
				_activityConfidences[x] = shtpData[11 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
			break;
		}
		case SHTP_REPORT_COMMAND_RESPONSE:
		{
			//printf("!");
			//The BNO08X responds with this report to command requests. It's up to use to remember which command we issued.
			uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

			if (command == COMMAND_ME_CALIBRATE)
			{
				//printf("ME Cal report found!");
				calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
			}
			break;
		}
		default:
		{
			//This sensor report ID is unhandled.
			//See reference manual to add additional feature reports as needed
		}
	}

	//TODO additional feature reports may be strung together. Parse them all.
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void BNO08X_getRollPitchYaw(float* roll, float* pitch, float* yaw)
{
    float dqw = BNO08X_getQuatReal();
    float dqx = BNO08X_getQuatI();
    float dqy = BNO08X_getQuatJ();
    float dqz = BNO08X_getQuatK();

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = 2.0 * (dqw * dqx + dqy * dqz);
    float t1 = 1.0 - 2.0 * (dqx * dqx + ysqr);
    *roll = atan2(t0, t1);

    // pitch (y-axis rotation)
	float t2 = 2.0 * (dqw * dqy - dqz * dqx);
	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;
	*pitch = asin(t2);

    // yaw (z-axis rotation)
	float t3 = 2.0 * (dqw * dqz + dqx * dqy);
	float t4 = 1.0 - 2.0 * (ysqr + dqz * dqz);
	*yaw = atan2(t3, t4);
}

//Return the rotation vector quaternion I
float BNO08X_getQuatI()
{
	return BNO08X_qToFloat(rawQuatI, rotationVector_Q1);
}

//Return the rotation vector quaternion J
float BNO08X_getQuatJ()
{
	return BNO08X_qToFloat(rawQuatJ, rotationVector_Q1);
}

//Return the rotation vector quaternion K
float BNO08X_getQuatK()
{
	return BNO08X_qToFloat(rawQuatK, rotationVector_Q1);
}

//Return the rotation vector quaternion Real
float BNO08X_getQuatReal()
{
	return BNO08X_qToFloat(rawQuatReal, rotationVector_Q1);
}

//Return the rotation vector accuracy
float BNO08X_getQuatRadianAccuracy()
{
	return BNO08X_qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
}

//Return the acceleration component
uint8_t BNO08X_getQuatAccuracy()
{
	return (quatAccuracy);
}

//Return the acceleration component
float BNO08X_getAccelX()
{
	return BNO08X_qToFloat(rawAccelX, accelerometer_Q1);
}

//Return the acceleration component
float BNO08X_getAccelY()
{
	return BNO08X_qToFloat(rawAccelY, accelerometer_Q1);
}

//Return the acceleration component
float BNO08X_getAccelZ()
{
	return BNO08X_qToFloat(rawAccelZ, accelerometer_Q1);
}

//Return the acceleration component
uint8_t BNO08X_getAccelAccuracy()
{
	return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Return the acceleration component
float BNO08X_getLinAccelX()
{
	return BNO08X_qToFloat(rawLinAccelX, linear_accelerometer_Q1);
}

//Return the acceleration component
float BNO08X_getLinAccelY()
{
	return BNO08X_qToFloat(rawLinAccelY, linear_accelerometer_Q1);
}

//Return the acceleration component
float BNO08X_getLinAccelZ()
{
	return BNO08X_qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
}

//Return the acceleration component
uint8_t BNO08X_getLinAccelAccuracy()
{
	return (accelLinAccuracy);
}

//Return the gyro component
float BNO08X_getGyroX()
{
	return BNO08X_qToFloat(rawGyroX, gyro_Q1);
}

//Return the gyro component
float BNO08X_getGyroY()
{
	return BNO08X_qToFloat(rawGyroY, gyro_Q1);
}

//Return the gyro component
float BNO08X_getGyroZ()
{
	return BNO08X_qToFloat(rawGyroZ, gyro_Q1);
}

//Return the gyro component
uint8_t BNO08X_getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Return the magnetometer component
float BNO08X_getMagX()
{
	return BNO08X_qToFloat(rawMagX, magnetometer_Q1);
}

//Return the magnetometer component
float BNO08X_getMagY()
{
	return BNO08X_qToFloat(rawMagY, magnetometer_Q1);
}

//Return the magnetometer component
float BNO08X_getMagZ()
{
	return BNO08X_qToFloat(rawMagZ, magnetometer_Q1);
}

//Return the mag component
uint8_t BNO08X_getMagAccuracy()
{
	return (magAccuracy);
}

//Return the step count
uint16_t BNO08X_getStepCount()
{
	return (stepCount);
}

//Return the stability classifier
uint8_t BNO08X_getStabilityClassifier()
{
	return (stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO08X_getActivityClassifier()
{
	return (activityClassifier);
}

//Return the time stamp
uint32_t BNO08X_getTimeStamp()
{
	return (timeStamp);
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO08X_getQ1(uint16_t recordID)
{
	//Q1 is always the lower 16 bits of word 7
	return BNO08X_readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO08X_getQ2(uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	return BNO08X_readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO08X_getQ3(uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	return BNO08X_readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO08X_getResolution(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = BNO08X_getQ1(recordID);

	//Resolution is always word 2
	uint32_t value = BNO08X_readFRSword(recordID, 2); //Get word 2

	return BNO08X_qToFloat(value, Q);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO08X_getRange(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = BNO08X_getQ1(recordID);

	//Range is always word 1
	uint32_t value = BNO08X_readFRSword(recordID, 1); //Get word 1

	return BNO08X_qToFloat(value, Q);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO08X_readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (BNO08X_readFRSdata(recordID, wordNumber, 1) == 1) //Get word number, just one word in length from FRS
		return (metaData[0]);						  //Return this one word

	return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO08X_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0;							//Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF;		//Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF;		//Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF;		//FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF;		//FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF;		//Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF;		//Block size MSB

	//Transmit packet on channel 2, 8 bytes
	BNO08X_sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
int BNO08X_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	//First we send a Flash Record System (FRS) request
	BNO08X_frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

	//Read bytes until FRS reports that the read is complete
	while (1)
	{
		//Now we wait for response
		while (1)
		{
			uint8_t counter = 0;
			while (BNO08X_receivePacket() == 0)
			{
				if (counter++ > 100)
					return (0); //Give up
				HAL_Delay(1);
			}

			//We have the packet, inspect it for the right contents
			//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
					break; //This packet is one we are looking for
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		//Record these words to the metaData array
		if (dataLength > 0)
		{
			metaData[spot++] = data0;
		}
		if (dataLength > 1)
		{
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE)
		{
			printf("metaData array over run. Returning.");
			return (1); //We have run out of space in our array. Bail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (1); //FRS status is read completed! We're done!
		}
	}
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO08X_softReset(void)
{
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	BNO08X_sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	HAL_Delay(50);
	while (BNO08X_receivePacket() == 1);
	HAL_Delay(50);
	while (BNO08X_receivePacket() == 1);
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO08X_resetReason()
{
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	BNO08X_sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (BNO08X_receivePacket() == 1)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (shtpData[1]);
		}
	}

	return (0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO08X_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	return fixedPointValue * powf(2, qPoint * -1);
}

//Sends the packet to enable the rotation vector
void BNO08X_enableRotationVector(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Sends the packet to enable the rotation vector
void BNO08X_enableGameRotationVector(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Sends the packet to enable the accelerometer
void BNO08X_enableAccelerometer(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

//Sends the packet to enable the accelerometer
void BNO08X_enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

//Sends the packet to enable the gyro
void BNO08X_enableGyro(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}

//Sends the packet to enable the magnetometer
void BNO08X_enableMagnetometer(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

//Sends the packet to enable the step counter
void BNO08X_enableStepCounter(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

//Sends the packet to enable the Stability Classifier
void BNO08X_enableStabilityClassifier(uint16_t timeBetweenReports)
{
	BNO08X_setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

//Sends the commands to begin calibration of the accelerometer
void BNO08X_calibrateAccelerometer()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO08X_calibrateGyro()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO08X_calibrateMagnetometer()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO08X_calibratePlanarAccelerometer()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO08X_calibrateAll()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO08X_endCalibration()
{
	BNO08X_sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
int BNO08X_calibrationComplete()
{
	if (calibrationStatus == 0)
		return (1);
	return (0);
}

//Given a sensor's report ID, this tells the BNO08X to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO08X_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig)
{
	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;						 //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;							 //Feature flags
	shtpData[3] = 0;							 //Change sensitivity (LSB)
	shtpData[4] = 0;							 //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;							 //Batch Interval (LSB)
	shtpData[10] = 0;							 //Batch Interval
	shtpData[11] = 0;							 //Batch Interval
	shtpData[12] = 0;							 //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   	 //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   	 //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	 //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	 //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	BNO08X_sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO08X_sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	BNO08X_sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO08X to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO08X_sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
	{
		;
	} //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	BNO08X_sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO08X
//See page 51 of reference manual
void BNO08X_requestCalibrationStatus()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	BNO08X_sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO08X to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO08X_saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	BNO08X_sendCommand(COMMAND_DCD); //Save DCD command
}

//Blocking wait for BNO08X to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
int BNO08X_waitForSPI(void)
{
	for (uint32_t counter = 0; counter < 0xffffffff; counter++) //Don't got more than 255
	{
        if (LL_GPIO_IsInputPinSet(_intPort, _intPin) == 0)
		{
			//printf("\nData available\n");
			return (1);
		}
		//printf("SPI Wait %d\n", counter);
	}
	printf("\nData not available\n");
	return (0);
}


//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
int BNO08X_receivePacket(void)
{
	uint8_t incoming;

    if (LL_GPIO_IsInputPinSet(_intPort, _intPin) == 1)
		return (0); //Data is not available

	//Old way: if (BNO08X_waitForSPI() == 0) return (0); //Something went wrong

	//Get first four bytes to find out how much data we need to read

    LL_GPIO_ResetOutputPin(_csPort, _csPin);

	//Get the first four bytes, aka the packet header
	uint8_t packetLSB = SPI2_SendByte(0);
	uint8_t packetMSB = SPI2_SendByte(0);
	uint8_t channelNumber = SPI2_SendByte(0);
	uint8_t seqNumber = SPI2_SendByte(0); //Not sure if we need to store this or not

	//Store the header info
	shtpHeader[0] = packetLSB;
	shtpHeader[1] = packetMSB;
	shtpHeader[2] = channelNumber;
	shtpHeader[3] = seqNumber;

	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
	dataLength &= 0x7fff; //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		return (0); //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	//printf("length: %d\n", dataLength);

	//Read incoming data into the shtpData array
	for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
	{
		incoming = SPI2_SendByte(0xFF);
		//printf("%d ", incoming);
		if (dataSpot < MAX_PACKET_SIZE)	//BNO08X can respond with upto 270 bytes, avoid overflow
			shtpData[dataSpot] = incoming; //Store data into the shtpData array
	}
	//printf("\n");

    LL_GPIO_SetOutputPin(_csPort, _csPin);
	return (1); //We're done!
}


//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
int BNO08X_sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for BNO08X to indicate it is available for communication
	if (BNO08X_waitForSPI() == 0)
		return (0); //Data is not available

	//BNO08X has max CLK of 3MHz, MSB first,
	//The BNO08X uses CPOL = 1 and CPHA = 1. This is mode3
    LL_GPIO_ResetOutputPin(_csPort, _csPin);

	//Send the 4 byte packet header
	SPI2_SendByte(packetLength & 0xFF);			//Packet length LSB
	SPI2_SendByte(packetLength >> 8);				//Packet length MSB
	SPI2_SendByte(channelNumber);					//Channel number
	SPI2_SendByte(sequenceNumber[channelNumber]++); 	//Send the sequence number, increments with each packet sent, different counter for each channel

	//Send the user's data packet
	for (uint8_t i = 0; i < dataLength; i++)
	{
		SPI2_SendByte(shtpData[i]);
	}

    LL_GPIO_SetOutputPin(_csPort, _csPin);

	return (1);
}
///////////////////////////////////////////////////////////////////////////



