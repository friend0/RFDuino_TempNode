/*
* @Author: Ryan A. Rodriguez
* @Date:   2015-10-31 22:24:49
*
* The IMU Node for the rfDuino is setup to broadcast IMU data over BLE to an iOS application
* This code is intended to run on the rfDuino module
*
* @Last Modified by:   Ryan A. Rodriguez
* @Last Modified time: 2015-10-31 23:10:49
*/



#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <RFduinoBLE.h>
#include <OneWire.h>
#include "libUBP.h"
#include "constants.h"
#include "data_types.h"
#include <Arduino.h>
#include "crc8.h"

////////////////
// I2C Setup //
///////////////


// SDO_XM and SDO_G are both grounded, therefore our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
/**
 * Create an instance of the LSM9DS0 library called `dof`
 * Params: [SPI or I2C Mode declaration], [gyro I2C address], [xm I2C address]
 */
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
MeasurementType aMeasurement, lastMeasurement;

///////////////////////
// Example SPI Setup //
///////////////////////

//#define LSM9DS0_CSG  9  // CSG connected to Arduino pin 9
//#define LSM9DS0_CSXM 10 // CSXM connected to Arduino pin 10
//LSM9DS0 dof(MODE_SPI, LSM9DS0_CSG, LSM9DS0_CSXM);

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
const byte INT1XM = 3; // INT1XM tells us when accel data is ready
const byte INT2XM = 2; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready

/**
 * Indicate whether raw or processed data output is desired
 * @type {Boolean}
 */
boolean printRaw = false;

/**
 * rfDuino spectific setup code.
 * Sets BLE parameters
 * @return {[type]} [description]
 */
void setupRFDuino()
{

        RFduinoBLE.deviceName = "RFduino";
        RFduinoBLE.advertisementData = "temp";
        RFduinoBLE.advertisementInterval = SECONDS(3);
        RFduinoBLE.txPowerLevel = 0;  // (-20dbM to +4 dBm)
        // Start the BLE stack
        RFduinoBLE.begin();

}

/**
 *  The setup function executes just once at startup
 */
void setup()
{

        // Set up interrupt pins as inputs:
        pinMode(INT1XM, INPUT);
        pinMode(INT2XM, INPUT);
        pinMode(DRDYG, INPUT);

        Serial.begin(9600); // Start serial at 115200 bps

        setupRFDuino(); // BLE setup

        // Use the begin() function to initialize the LSM9DS0 library.
        // You can either call it with no parameters (the easy way):
        uint16_t status = dof.begin();
        // Or call it with declarations for sensor scales and data rates:
        //uint16_t status = dof.begin(dof.G_SCALE_2000DPS, dof.A_SCALE_6G, dof.M_SCALE_2GS);

        // begin() returns a 16-bit value which includes both the gyro and
        // accelerometers WHO_AM_I response. You can check this to make sure
        // communication was successful.
        Serial.println(status, HEX);

        // todo: this should use and arra with memcpy or some such
        // BUT, I think this will take some doing, i.e. some mods to slipBuffer
        //current =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        //last = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        //memcpy(&aMeasurement., &array[0], sizeof(my_structure.one));

        aMeasurement.A = 0;
        aMeasurement.B = 0;
        aMeasurement.C = 0;
        aMeasurement.D = 0;
        aMeasurement.E = 0;
        aMeasurement.F = 0;
        aMeasurement.G = 0;
        aMeasurement.H = 0;
        aMeasurement.Z = 0;
        aMeasurement.J = 0;
        aMeasurement.K = 0;
        aMeasurement.L = 0;

}

/**
 * Code in loop will be executed continuously
 */
void loop()
{

        stream_all();

        // todo: Need to implement this logic here! ONly send a new packet if the info differs from the last one
        //aMeasurement != lastMeasurement

        if (1) {

                bool success = UBP_queuePacketTransmission(MEASUREMENT_v1, UBP_TxFlagNone, (char *) &aMeasurement, sizeof(MeasurementType));

                if (success) {
                        // put your main code here, to run repeatedly:
                        while (UBP_isBusy() == true) UBP_pump();

                        RFduino_ULPDelay( MILLISECONDS(250) );
                        lastMeasurement = aMeasurement;
                } else {
                        Serial.println("Failed to enqueue packet");
                }
        }
}

/**
 * Process accelerometer data
 */
void print_accel()
{
        // Only read from the accelerometer if the accel interrupts,
        // which means that new data is ready.
        if (digitalRead(INT1XM))
        {

                dof.readAccel();

                if (printRaw) {
                        // If you want to print calculated values, you can use the
                        // calcAccel helper function to convert a raw ADC value to
                        // g's. Give the function the value that you want to convert.

                        aMeasurement.A = dof.ax;
                        aMeasurement.B = dof.ay;
                        aMeasurement.C = dof.az;

                }
                else {
                        float x = dof.calcAccel(dof.ax);
                        float y = dof.calcAccel(dof.ay);
                        float z = dof.calcAccel(dof.az);

                        aMeasurement.A = x;
                        aMeasurement.B = y;
                        aMeasurement.C = z;

                }


        }
}


/**
 * Process gyroscope data
 */
void print_gyro()
{
        // Only read from the gyro if the DRDY interrupts,
        // which means that new data is ready.
        if (digitalRead(DRDYG))
        {
                // Use the readGyro() function to get new data from the gyro.
                // After calling this function, new values will be stored in
                // the gx, gy, and gz variables.
                dof.readGyro();

                Serial.print("G: ");
                if (printRaw){

                        aMeasurement.G = dof.gx;
                        aMeasurement.H = dof.gy;
                        aMeasurement.Z = dof.gz;                        
                } else {
                        // Using the calcGyro helper function, we can get the
                        // gyroscope readings in degrees per second (DPS).

                        float x = dof.calcGyro(dof.gx);
                        float y = dof.calcGyro(dof.gy);
                        float z = dof.calcGyro(dof.gz);

                        aMeasurement.G = x;
                        aMeasurement.H = y;
                        aMeasurement.Z = z;

                        //Serial.print(x);
                        //Serial.print(", ");
                        //Serial.print(y);
                        //Serial.print(", ");
                        //Serial.println(z);
                }
        }
}

/**
 * Process magnetometer data
 */
void print_mag()
{
        // Only read from the magnetometer if the INT2XM interrupts,
        // which means that new data is ready.
        if (digitalRead(INT2XM))
        {
                // Use the readMag() function to get new data from the mag.
                // After calling this function, new values will be stored in
                // the mx, my, and mz variables.
                dof.readMag();

                Serial.print("M: ");
                if (printRaw) {
                        Serial.print(dof.mx);
                        Serial.print(", ");
                        Serial.print(dof.my);
                        Serial.print(", ");
                        Serial.print(dof.mz);
                        Serial.print(", ");
                        Serial.println(calc_heading(dof.mx, dof.my));
                } else {
                        float x = dof.calcMag(dof.mx);
                        float y = dof.calcMag(dof.my);
                        float z = dof.calcMag(dof.mz);

                        aMeasurement.D = x;
                        aMeasurement.E = y;
                        aMeasurement.F = z;

                        // Using the calcMg helper function, we can get the
                        // magnetometer readings in gauss (Gs).
                        //Serial.print(x, 4);
                        //Serial.print(", ");
                        //Serial.print(y, 4);
                        //Serial.print(", ");
                        //Serial.print(z, 4);
                        //Serial.print(", ");
                        //Serial.println(calc_heading(y, z));
                }
        }
}


/**
 * Here's a simple example function to calculate heading based on magnetometer readings. This only works when the 9DOF is flat
 * (x-axis normal to gravity).
 *
 * @param  hx [description]
 * @param  hy [description]
 * @return    [description]
 */
float calc_heading(float hx, float hy)
{
        if (hy > 0) {
                return 90 - (atan(hx / hy) * 180 / PI);
        } else if (hy < 0) {
                return 270 - (atan(hx / hy) * 180 / PI);
        } else // hy = 0 {
                if (hx < 0) return 180;
                else return 0;
        }
}


/**
 * Start looking for interrupts on all three channels, attach them to the SLIP packet to be sent out
 */
void stream_all()
{
        if ((digitalRead(INT2XM)) && (digitalRead(INT1XM)) && (digitalRead(DRDYG))) {
                print_accel();
                print_gyro();
                print_mag();
        }
}



/**
 * Provides an interface to switch the full-scale range of each sensor. This function will block until three characters
 * (to select the three ranges) are received.
 */
void setScale()
{
        char c;

        // Print the accelerometer range options:
        Serial.println(F("Set accelerometer scale:"));
        Serial.println(F("\t1) +/- 2G"));
        Serial.println(F("\t2) +/- 4G"));
        Serial.println(F("\t3) +/- 6G"));
        Serial.println(F("\t4) +/- 8G"));
        Serial.println(F("\t5) +/- 16G"));
        // Wait for a serial char to come in:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setAccelScale function to set the accelerometer
        // full-scale range to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setAccelScale(dof.A_SCALE_2G);
                break;
        case '2':
                dof.setAccelScale(dof.A_SCALE_4G);
                break;
        case '3':
                dof.setAccelScale(dof.A_SCALE_6G);
                break;
        case '4':
                dof.setAccelScale(dof.A_SCALE_8G);
                break;
        case '5':
                dof.setAccelScale(dof.A_SCALE_16G);
                break;
        }
        // Print the gyro scale ranges:
        Serial.println(F("Set gyroscope scale:"));
        Serial.println(F("\t1) +/- 245 DPS"));
        Serial.println(F("\t2) +/- 500 DPS"));
        Serial.println(F("\t3) +/- 2000 DPS"));
        // Wait for a character to come in:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setGyroScale function to set the gyroscope
        // full-scale range to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setGyroScale(dof.G_SCALE_245DPS);
                break;
        case '2':
                dof.setGyroScale(dof.G_SCALE_500DPS);
                break;
        case '3':
                dof.setGyroScale(dof.G_SCALE_2000DPS);
                break;
        }
        // Print the magnetometer scale options:
        Serial.println(F("Set magnetometer scale:"));
        Serial.println(F("\t1) +/- 2GS"));
        Serial.println(F("\t2) +/- 4GS"));
        Serial.println(F("\t3) +/- 8GS"));
        Serial.println(F("\t4) +/- 12GS"));
        // Wait for a char:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setMagScale function to set the magnetometer
        // full-scale range to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setMagScale(dof.M_SCALE_2GS);
                break;
        case '2':
                dof.setMagScale(dof.M_SCALE_4GS);
                break;
        case '3':
                dof.setMagScale(dof.M_SCALE_8GS);
                break;
        case '4':
                dof.setMagScale(dof.M_SCALE_12GS);
                break;
        }
}

// set_raw simply switches the state of the global printRaw
// variable. It'll print a message to say what it's switched to.
void set_raw()
{
        if (printRaw)
        {
                printRaw = false;
                Serial.println(F("Printing calculated readings"));
        }
        else
        {
                printRaw = true;
                Serial.println(F("Printing raw readings"));
        }
}

// setODR() provides a serial interface to set the output data
// rate (ODR) for each sensor. It will block until it receives
// three characters to set the data rates.
void setODR()
{
        char c;

        // Print the menu options for accel data rate:
        Serial.println(F("Set Accelerometer ODR (Hz):"));
        Serial.println(F("\t1) 3.125 \t 6) 100"));
        Serial.println(F("\t2) 6.25  \t 7) 200"));
        Serial.println(F("\t3) 12.5  \t 8) 400"));
        Serial.println(F("\t4) 25    \t 9) 800"));
        Serial.println(F("\t5) 50    \t A) 1600"));
        // Wait for a character to be read in:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setAccelODR function to set the accelerometer
        // data rate to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setAccelODR(dof.A_ODR_3125);
                break;
        case '2':
                dof.setAccelODR(dof.A_ODR_625);
                break;
        case '3':
                dof.setAccelODR(dof.A_ODR_125);
                break;
        case '4':
                dof.setAccelODR(dof.A_ODR_25);
                break;
        case '5':
                dof.setAccelODR(dof.A_ODR_50);
                break;
        case '6':
                dof.setAccelODR(dof.A_ODR_100);
                break;
        case '7':
                dof.setAccelODR(dof.A_ODR_200);
                break;
        case '8':
                dof.setAccelODR(dof.A_ODR_400);
                break;
        case '9':
                dof.setAccelODR(dof.A_ODR_800);
                break;
        case 'A':
        case 'a':
                dof.setAccelODR(dof.A_ODR_1600);
                break;
        }

        // Print the menu options for the gyro ODR's
        Serial.println(F("Set Gyro ODR/Cutoff (Hz):"));
        Serial.println(F("\t1) 95/12.5 \t 8) 380/25"));
        Serial.println(F("\t2) 95/25   \t 9) 380/50"));
        Serial.println(F("\t3) 190/125 \t A) 380/100"));
        Serial.println(F("\t4) 190/25  \t B) 760/30"));
        Serial.println(F("\t5) 190/50  \t C) 760/35"));
        Serial.println(F("\t6) 190/70  \t D) 760/50"));
        Serial.println(F("\t7) 380/20  \t E) 760/100"));
        // Wait for a character to arrive:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setGyroODR function to set the gyroscope
        // data rate to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setGyroODR(dof.G_ODR_95_BW_125);
                break;
        case '2':
                dof.setGyroODR(dof.G_ODR_95_BW_25);
                break;
        case '3':
                dof.setGyroODR(dof.G_ODR_190_BW_125);
                break;
        case '4':
                dof.setGyroODR(dof.G_ODR_190_BW_25);
                break;
        case '5':
                dof.setGyroODR(dof.G_ODR_190_BW_50);
                break;
        case '6':
                dof.setGyroODR(dof.G_ODR_190_BW_70);
                break;
        case '7':
                dof.setGyroODR(dof.G_ODR_380_BW_20);
                break;
        case '8':
                dof.setGyroODR(dof.G_ODR_380_BW_25);
                break;
        case '9':
                dof.setGyroODR(dof.G_ODR_380_BW_50);
                break;
        case 'A':
        case 'a':
                dof.setGyroODR(dof.G_ODR_380_BW_100);
                break;
        case 'B':
        case 'b':
                dof.setGyroODR(dof.G_ODR_760_BW_30);
                break;
        case 'C':
        case 'c':
                dof.setGyroODR(dof.G_ODR_760_BW_35);
                break;
        case 'D':
        case 'd':
                dof.setGyroODR(dof.G_ODR_760_BW_50);
                break;
        case 'E':
        case 'e':
                dof.setGyroODR(dof.G_ODR_760_BW_100);
                break;
        }

        // Print all possible range selections for the magnetometer:
        Serial.println(F("Set Magnetometer ODR (Hz):"));
        Serial.println(F("\t1) 3.125 \t 4) 25"));
        Serial.println(F("\t2) 6.25  \t 5) 50"));
        Serial.println(F("\t3) 12.5  \t 6) 100"));
        // Wait for a character to come in:
        while (Serial.available() < 1)
                ;
        c = Serial.read();
        // Use the setMagODR function to set the magnetometer
        // data rate to any of the possible ranges. These ranges
        // are all defined in SFE_LSM9DS0.h.
        switch (c)
        {
        case '1':
                dof.setMagODR(dof.M_ODR_3125);
                break;
        case '2':
                dof.setMagODR(dof.M_ODR_625);
                break;
        case '3':
                dof.setMagODR(dof.M_ODR_125);
                break;
        case '4':
                dof.setMagODR(dof.M_ODR_25);
                break;
        case '5':
                dof.setMagODR(dof.M_ODR_50);
                break;
        case '6':
                dof.setMagODR(dof.M_ODR_100);
                break;
        }
}

/**
 * Print the TUI menu used by the user application
 */
void print_menu()
{
        Serial.println();
        Serial.println(F("////////////////////////////////////////////"));
        Serial.println(F("// LSM9DS0 Super Awesome Amazing Fun Time //"));
        Serial.println(F("////////////////////////////////////////////"));
        Serial.println();
        Serial.println(F("1) Stream Accelerometer"));
        Serial.println(F("2) Stream Gyroscope"));
        Serial.println(F("3) Stream Magnetometer"));
        Serial.println(F("4) Stream output from all sensors"));
        Serial.println(F("5) Set Sensor Scales"));
        Serial.println(F("6) Switch To/From Raw/Calculated Readings"));
        Serial.println(F("7) Set Output Data Rates"));
        Serial.println();
}


/**
 * TUI menu function for when the chip is being used cirectly from PC. Through a serial monitor, the user can interface with the
 * IMU and set configurations, queue streaming and etc.
 *
 * @param c input from user
 */
void parse_menu(char c)
{
        switch (c)
        {
        case '1':
                while (!Serial.available())
                        print_accel(); // Print accelerometer values
                break;
        case '2':
                while (!Serial.available())
                        print_gyro(); // Print gyroscope values
                break;
        case '3':
                while (!Serial.available())
                        print_mag(); // Print magnetometer values
                break;
        case '4':
                while (!Serial.available())
                        stream_all(); // Print all sensor readings
                break;
        case '5':
                setScale(); // Set the ranges of each sensor
                break;
        case '6':
                set_raw(); // Switch between calculated and raw output
                break;
        case '7':
                setODR(); // Set the data rates of each sensor
                break;
        }
}

/**
 * Callback function for UBP SLIP buffer packet.
 *
 * @param packetIdentifier Packet ID value
 * @param txFlags          transmission flags
 * @param packetBuffer     packet buffer
 */
void UBP_receivedPacket(unsigned short packetIdentifier, UBP_TxFlags txFlags, void *packetBuffer) {

        switch (packetIdentifier) {
        case MEASUREMENT_v1: {
                break;
        }
        }

}

/**
 * A callback function for handling received packets. In this case, packets are received from iOS.
 * As of now, received packets are not used for anything. In the future, these ight be used to setup IMU configurations from
 * the iOS application.
 *
 * @param data data packet received
 * @param len  length of data packet
 */
void RFduinoBLE_onReceive(char *data, int len) {

        // if the first byte is 0x01 / on / true
        if (data[0] == 1) {
                ;
        }
        else if (data[0] == 0) {
                ;
        }

}

