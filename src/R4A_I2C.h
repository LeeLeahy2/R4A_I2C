/**********************************************************************
  R4A_I2C.h

  Declare the I2C support
**********************************************************************/

#ifndef __R4A_I2C_H__
#define __R4A_I2C_H__

#include <Arduino.h>            // Built-in
#include <Wire.h>               // Built-in

#include <R4A_Common.h>          // Robots-For-All common support

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
    //   display: Device used for debug output
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    // Outputs:
    //   Returns the number of bytes read
    virtual size_t read(uint8_t deviceI2cAddress,
                        const uint8_t * cmdBuffer, // Does not include I2C address
                        size_t cmdByteCount,
                        uint8_t * readBuffer,
                        size_t readByteCount,
                        Print * display = nullptr,
                        bool releaseI2cBus = true);

    // Send data to an I2C peripheral
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    //   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
    //   cmdByteCount: Number of bytes to send from the command buffer
    //   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
    //   dataByteCount: Number of bytes to send from the data buffer
    //   display: Device used for debug output
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    // Outputs:
    //   Returns true upon success, false otherwise
    bool write(uint8_t deviceI2cAddress,
               const uint8_t * cmdBuffer,
               size_t cmdByteCount,
               const uint8_t * dataBuffer,
               size_t dataByteCount,
               Print * display = nullptr,
               bool releaseI2cBus = true);

  private:

    // Send data to an I2C peripheral, entered with the I2C bus lock held
    // Inputs:
    //   deviceAddress: Device address on the I2C bus (0 - 0x7f)
    //   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
    //   cmdByteCount: Number of bytes to send from the command buffer
    //   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
    //   dataByteCount: Number of bytes to send from the data buffer
    //   display: Device used for debug output
    //   releaseI2cBus: A value of true releases the I2C bus after the transaction
    // Outputs:
    //   Returns true upon success, false otherwise
    virtual bool writeWithLock(uint8_t deviceI2cAddress,
                               const uint8_t * cmdBuffer,
                               size_t cmdByteCount,
                               const uint8_t * dataBuffer,
                               size_t dataByteCount,
                               Print * display = nullptr,
                               bool releaseI2cBus = true);
};

#endif  // R4A_USING_ESP32
