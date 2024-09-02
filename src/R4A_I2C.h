/**********************************************************************
  R4A_I2C.h

  Declare the I2C support
**********************************************************************/

#ifndef __R4A_I2C_H__
#define __R4A_I2C_H__

#include <Arduino.h>            // Built-in
#include <Wire.h>               // Built-in

#include <R4A_Common.h>

//****************************************
// Constants
//****************************************

// I2C General call values
const uint8_t R4A_I2C_GENERAL_CALL_DEVICE_ADDRESS = 0x00;
const uint8_t R4A_I2C_SWRST = 0x06;

// I2C bus speeds
#define R4A_I2C_STANDARD_MODE_HZ        (100 * 1000)        // 100 KHz
#define R4A_I2C_FAST_MODE_HZ            (400 * 1000)        // 400 KHZ
#define R4A_I2C_FAST_MODE_PLUS_HZ       (1 * 1000 * 1000)   // 1.0 MHz
#define R4A_I2C_HIGH_SPEED_MODE_HZ      (34 * 100 * 1000)   // 3.4 MHz

//****************************************
// Generic I2C Class
//****************************************

// I2C device description
typedef struct _R4A_I2C_DEVICE_DESCRIPTION
{
    uint8_t deviceAddress;  // I2C device address: 0 - 0x7f
    char * displayName;     // Name to display when the device is found
} R4A_I2C_DEVICE_DESCRIPTION;

// I2C bus
class R4A_I2C_BUS
{
  protected:

    TwoWire * _i2cBus;  // API for the I2C bus
    const R4A_I2C_DEVICE_DESCRIPTION * const _deviceTable; // I2C device table
    const int _deviceTableEntries;    // Number of entries in the I2C device table
    volatile int _lock; // Lock to synchronize access to the I2C bus

  public:

    // Create the R4A_I2C object
    // Inputs:
    //   deviceTable: Address of the table containing the address and device
    //                descriptions, may be nullptr
    //   deviceTableEntries: Number of entries in the I2C device table
    R4A_I2C_BUS(const R4A_I2C_DEVICE_DESCRIPTION * deviceTable,
                int deviceTableEntries)
        : _deviceTable{deviceTable}, _deviceTableEntries{deviceTableEntries}
    {
    }

    // Delete the object
    ~R4A_I2C_BUS()
    {
    }

    // Enumerate the I2C bus
    // Inputs:
    //   display: Device used for output
    void enumerate(Print * display = &Serial);

    // Ping an I2C device and see if it responds
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    // Outputs:
    //   Returns true if device detected, false otherwise
    bool isDevicePresent(uint8_t deviceAddress);

    // Read data from an I2C peripheral
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    //   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
    //   cmdByteCount: Number of bytes to send from the command buffer
    //   dataBuffer: Address of the buffer to receive the data bytes, may be nullptr
    //   dataByteCount: Size in bytes of the data buffer, maximum receive bytes
    //   debug: A true value enables debugging for the I2C transaction
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    //   display: Device used for output
    // Outputs:
    //   Returns the number of bytes read
    virtual size_t read(uint8_t deviceI2cAddress,
                        const uint8_t * cmdBuffer, // Does not include I2C address
                        size_t cmdByteCount,
                        uint8_t * readBuffer,
                        size_t readByteCount,
                        bool debug = false,
                        bool releaseI2cBus = true,
                        Print * display = &Serial);

    // Send data to an I2C peripheral
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    //   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
    //   cmdByteCount: Number of bytes to send from the command buffer
    //   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
    //   dataByteCount: Number of bytes to send from the data buffer
    //   debug: A true value enables debugging for the I2C transaction
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    //   display: Device used for output
    // Outputs:
    //   Returns true upon success, false otherwise
    bool write(uint8_t deviceI2cAddress,
               const uint8_t * cmdBuffer,
               size_t cmdByteCount,
               const uint8_t * dataBuffer,
               size_t dataByteCount,
               bool debug = false,
               bool releaseI2cBus = true,
               Print * display = &Serial);

  private:

    // Send data to an I2C peripheral, entered with the I2C bus lock held
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    //   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
    //   cmdByteCount: Number of bytes to send from the command buffer
    //   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
    //   dataByteCount: Number of bytes to send from the data buffer
    //   debug: A true value enables debugging for the I2C transaction
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    //   display: Device used for output
    // Outputs:
    //   Returns true upon success, false otherwise
    virtual bool writeWithLock(uint8_t deviceI2cAddress,
                               const uint8_t * cmdBuffer,
                               size_t cmdByteCount,
                               const uint8_t * dataBuffer,
                               size_t dataByteCount,
                               bool debug = false,
                               bool releaseI2cBus = true,
                               Print * display = &Serial);
};

//****************************************
// PA9685 API
//****************************************

#define R4A_PCA9685_CHANNEL_COUNT       16

#define R4A_PCA9685_CHANNEL_0       0
#define R4A_PCA9685_CHANNEL_1       1
#define R4A_PCA9685_CHANNEL_2       2
#define R4A_PCA9685_CHANNEL_3       3
#define R4A_PCA9685_CHANNEL_4       4
#define R4A_PCA9685_CHANNEL_5       5
#define R4A_PCA9685_CHANNEL_6       6
#define R4A_PCA9685_CHANNEL_7       7
#define R4A_PCA9685_CHANNEL_8       8
#define R4A_PCA9685_CHANNEL_9       9
#define R4A_PCA9685_CHANNEL_10      10
#define R4A_PCA9685_CHANNEL_11      11
#define R4A_PCA9685_CHANNEL_12      12
#define R4A_PCA9685_CHANNEL_13      13
#define R4A_PCA9685_CHANNEL_14      14
#define R4A_PCA9685_CHANNEL_15      15

#define R4A_PCA9685_REGS_PER_CHANNEL        4

class R4A_PCA9685
{
private:

    uint8_t   _channelRegs[R4A_PCA9685_CHANNEL_COUNT << 2];
    uint32_t  _clockHz;
    uint32_t  _externalClockHz;
    R4A_I2C_BUS * _i2cBus;
    uint8_t   _i2cAddress;
    uint16_t  _max[R4A_PCA9685_CHANNEL_COUNT];
    uint16_t  _min[R4A_PCA9685_CHANNEL_COUNT];

public:

    // Constructor
    // Inputs:
    //   i2cBus: Address of an R4A_I2C object
    //   i2cAddress: Address of the PA9685 on the I2C bus
    //   scanClockHertz: Approximate frequency to scan the LEDs (23 - 1525)
    //   externalClockHertz: Frequency of external clock, zero (0) for internal clock
    R4A_PCA9685(R4A_I2C_BUS * i2cBus,
                uint8_t i2cAddress,
                uint32_t scanClockHertz,
                uint32_t externalClockHertz = 25 * 1000 * 1000);

    // Initialize the PA9685 LED controller
    // Output:
    //   Returns true if successful, false otherwise
    bool begin();

    // Buffer a copy of the LED on and off times which will be written to the
    // PCA9685 at a later time
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   onTime: Value between 0 and 4096, Time = value / (4096 * PA9685 frequency Hz)
    //   display: Device used for output
    // Outputs:
    //   Returns true if successful and false otherwise
    bool bufferLedOnOff(uint8_t channel,
                        uint16_t onTime,
                        Print * display = &Serial);

    // Buffer a copy of the servo position will be written to the PCA9685 at
    // a later time
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   degrees: Value between 0 and 180
    //   display: Device used for output
    // Outputs:
    //   Returns true if successful and false otherwise
    bool bufferServoPosition(uint8_t channel,
                             uint8_t degrees,
                             Print * display = &Serial);

    // Convert channel number into a PCA9685 register address
    // Inputs:
    //   channel: Channel number (0 - 15)
    // Outputs:
    //   Returns the first PA9685 register address for the channel
    uint8_t channelToRegister(uint8_t channel);

    // Copy the buffered register data into another buffer
    // Inputs:
    //   destBuffer: Address of the buffer to receive a copy of the registers
    void copyBufferedRegisters(uint8_t * destBuffer);

    // Display the current state of the PCA9685 channel
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   display: Device used for output
    void displayLedOnOff(uint8_t channel,
                         Print * display = &Serial);

    // Display the PCA9685 mode registers
    // Inputs:
    //   display: Device used for output
    void displayRegisters(Print * display = &Serial);

    // Dump all of the PCA9685 registers in hexadecimal
    // Inputs:
    //   display: Device used for output
    void dumpRegisters(Print * display = &Serial);

    // Set the LED on and off times
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   onTime: Value between 0 and 4096, Time = value / (4096 * PA9685 frequency Hz)
    //   display: Device used for output
    //   debug: Debug the I2C transactions
    // Outputs:
    //   Returns true if successful and false otherwise
    bool ledOnOff(uint8_t channel,
                  uint16_t onTime,
                  Print * display = &Serial,
                  bool debug = false);

    // Read one or more PCA9685 registers
    // Inputs:
    //   firstRegisterAddress: Address of the first PA9685 register to be read
    //   dataBuffer: Address of the buffer to receive the register values
    //   dataByteCount: Number of bytes to read from the PA9685 device
    //   display: Device used for output
    //   debug: Debug this I2C transaction
    // Outputs:
    //   Returns the number of bytes read
    size_t readRegisters(uint8_t firstRegisterAddress,
                         uint8_t * dataBuffer,
                         size_t dataByteCount,
                         Print * display = &Serial,
                         bool debug = false);

    // Convert from degrees (0 - 180) to onTime for servo positioning
    // Inputs:
    //   degrees:  Servo position in degrees (0 - 180)
    // Outputs:
    //   Return the onTicks to program into the PCA9685
    uint16_t servoDegreesToOnTicks(uint8_t degrees);

    // Convert from onTime for servo positioning to degrees (0 - 180)
    // Inputs:
    //   onTicks: Ticks for onTime programmed into the PCA9685
    // Outputs:
    //   Returns the degrees for the servo position
    uint8_t servoOnTicksToDegrees(uint16_t onTime);

    // Set the servo position
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   degrees: Value between 0 and 180
    //   display: Device used for output
    //   debug: Debug the I2C transactions
    // Outputs:
    //   Returns true if successful and false otherwise
    bool servoPosition(uint8_t channel,
                       uint8_t degrees,
                       Print * display = &Serial,
                       bool debug = false);

    // Write the buffered register data to the PCB9685 registers
    // Inputs:
    //   firstRegisterAddress: Address of the first PA9685 register to write
    //   dataByteCount: Number of bytes to write to the PA9685 device
    //   display: Device used for output
    //   debug: Debug this I2C transaction
    // Outputs:
    //   Returns true if successful, false otherwise
    bool writeBufferedRegisters(uint8_t firstRegisterAddress,
                                size_t dataByteCount,
                                Print * display = &Serial,
                                bool debug = false);

    // Write data to the PCB9685 registers
    // Inputs:
    //   firstRegisterAddress: Address of the first PA9685 register to write
    //   dataBuffer: Address of the buffer containing the data to write
    //   dataByteCount: Number of bytes to write to the PA9685 device
    //   display: Device used for output
    //   debug: Debug this I2C transaction
    // Outputs:
    //   Returns true if successful, false otherwise
    bool writeRegisters(uint8_t firstRegisterAddress,
                        uint8_t * dataBuffer,
                        size_t dataByteCount,
                        Print * display = &Serial,
                        bool debug = false);

  private:

    // Display mode 1 register
    // Inputs:
    //   display: Device used for output
    void displayMode1(Print * display = &Serial);

    // Display mode 2 register
    // Inputs:
    //   display: Device used for output
    void displayMode2(Print * display = &Serial);
};

#endif  // R4A_USING_ESP32
