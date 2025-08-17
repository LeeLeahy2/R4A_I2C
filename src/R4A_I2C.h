/**********************************************************************
  R4A_I2C.h

  Robots-For-All (R4A)
  Declare the I2C support
**********************************************************************/

#ifndef __R4A_I2C_H__
#define __R4A_I2C_H__

#include <Arduino.h>            // Built-in

#include <R4A_Robot.h>          // Robots-For-All robot support
#include <R4A_SX1509.h>         // Robots-For-All SX1509 support

// External libraries
#include <SparkFun_u-blox_GNSS_v3.h>    //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3

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

#define R4A_I2C_ADDRESSES_8_BIT      128    // Total 7-bit addresses available on the I2C bus
#define R4A_I2C_ADDRESSES_10_BIT    1024    // Total 10-bit addresses available on the I2C bus
#define R4A_I2C_ADDRESSES       R4A_I2C_ADDRESSES_10_BIT

//****************************************
// Forward data structures
//****************************************

#if (R4A_I2C_ADDRESSES == 128)
typedef uint8_t R4A_I2C_ADDRESS_t;  // 0 - 127 (0x7f), 7-bit address
#else
typedef uint16_t R4A_I2C_ADDRESS_t; // 0 - 1023 (0x3ff), 10-bit address
#endif

//****************************************
// Camera
//****************************************

typedef struct _R4A_CAMERA_PINS
{
    // Control pins
    int _pinReset;
    int _pinPowerDown;
    int _pinXCLK;       // External clock input
    int _pinStrobe;     // Flash control output

    // Serial Camera Control Bus (SCCB/I2C) pins
    int _pinSccbClk;
    int _pinSccbData;

    // Frame synchronization
    int _pinVSYNC;  // High at beginning of frame
    int _pinHREF;   // High during each horizontal line
    int _pinPCLK;   // Pixel clock output

    // Image data pins
    int _pinY0;
    int _pinY1;
    int _pinY2;
    int _pinY3;
    int _pinY4;
    int _pinY5;
    int _pinY6;
    int _pinY7;
    int _pinY8;
    int _pinY9;
} R4A_CAMERA_PINS;

// Define the frame formats
enum R4A_FRAME_SIZE_t
{
    R4A_FRAME_SIZE_40x30 = 0,   //  0:   40 x   30
    R4A_FRAME_SIZE_80x60,       //  1;   80 x   60
    R4A_FRAME_SIZE_96x96,       //  2:   96 x   96
    R4A_FRAME_SIZE_100x75,      //  3:  100 x   75
    R4A_FRAME_SIZE_128x128,     //  4:  128 x  128
    R4A_FRAME_SIZE_QQVGA,       //  5:  160 x  120
    R4A_FRAME_SIZE_QCIF,        //  6:  176 x  144
    R4A_FRAME_SIZE_200x150,     //  7:  200 x  150
    R4A_FRAME_SIZE_HQVGA,       //  8:  240 x  176
    R4A_FRAME_SIZE_240x240,     //  9:  240 x  240
    R4A_FRAME_SIZE_QVGA,        // 10:  320 x  240
    R4A_FRAME_SIZE_320x320,     // 11:  320 x  320
    R4A_FRAME_SIZE_CIF,         // 12:  400 x  296
    R4A_FRAME_SIZE_400x300,     // 13:  400 x  300
    R4A_FRAME_SIZE_HVGA,        // 14:  480 x  320
    R4A_FRAME_SIZE_VGA,         // 15:  640 x  480
    R4A_FRAME_SIZE_P_HD,        // 16:  720 x 1280
    R4A_FRAME_SIZE_SVGA,        // 17:  800 x  600
    R4A_FRAME_SIZE_P_3MP,       // 18:  864 x 1536
    R4A_FRAME_SIZE_XGA,         // 19: 1024 x  768
    R4A_FRAME_SIZE_P_FHD,       // 20: 1080 x 1920
    R4A_FRAME_SIZE_HD,          // 21: 1280 x  720
    R4A_FRAME_SIZE_SXGA,        // 22: 1280 x 1024
    R4A_FRAME_SIZE_UXGA,        // 23: 1600 x 1200
    R4A_FRAME_SIZE_FHD,         // 24: 1920 x 1080
    R4A_FRAME_SIZE_QXGA,        // 25: 2048 x 1536
    R4A_FRAME_SIZE_QHD,         // 26: 2560 x 1440
    R4A_FRAME_SIZE_WQXGA,       // 27: 2560 x 1600
    R4A_FRAME_SIZE_QSXGA,       // 28: 2560 x 1920
    R4A_FRAME_SIZE_5MP,         // 29: 2592 x 1944
    // Add new frame sizes above this line
    R4A_FRAME_SIZE_MAX
};

// One bit for each of the R4A_FRAM_SIZE_t values
typedef  uint32_t R4A_FRAME_SIZE_MASK_t;

// Define the camera frame data structure
typedef struct _R4A_CAMERA_FRAME
{
    uint16_t _xPixels;           // Number of pixels in the X axis
    uint16_t _yPixels;           // Number of pixels in the Y axis
    const char * _name;          // Name of this frame size
    R4A_FRAME_SIZE_t _frameSize; // One of the R4A_FRAME_SIZE_* values
} R4A_CAMERA_FRAME;

// Define the pixel formats
enum R4A_PIXEL_FORMAT_t
{
    R4A_PIXEL_FORMAT_GRAYSCALE = 0, //  0
    R4A_PIXEL_FORMAT_JPEG,          //  1
    R4A_PIXEL_FORMAT_RAW,           //  2
    R4A_PIXEL_FORMAT_RGB444,        //  3
    R4A_PIXEL_FORMAT_RGB555,        //  4
    R4A_PIXEL_FORMAT_RGB565,        //  5
    R4A_PIXEL_FORMAT_RGB888,        //  6
    R4A_PIXEL_FORMAT_YUV420,        //  7
    R4A_PIXEL_FORMAT_YUV422,        //  8
    // Add new pixel formats above this line
    R4A_PIXEL_FORMAT_MAX
};

// One bit for each of the R4A_PIXEL_FORMAT_t values
typedef  uint16_t R4A_PIXEL_FORMAT_MASK_t;

// Define the camera pixel format data structure
typedef struct _R4A_CAMERA_PIXEL
{
    const char * _name;         // Name of this pixel format
    bool _color;                // True if color image, false if B/W or grayscale
    uint8_t _bitsPerPixel;      // Number of bits for each pixel
    R4A_PIXEL_FORMAT_t _format; // Format of the pixels
} R4A_CAMERA_PIXEL;

extern const R4A_CAMERA_FRAME r4aCameraFrameFormats[];
extern const int r4aCameraFrameFormatsEntries;
extern const R4A_CAMERA_PIXEL r4aCameraPixelFormats[];
extern const int r4aCameraPixelFormatsEntries;

// Lookup the frame size
// Inputs:
//   frameSize: One of the R4A_FRAME_SIZE_* values
// Outputs:
//   Returns the address of a R4A_CAMERA_FRAME data structure if found or
//   nullptr when not found
const R4A_CAMERA_FRAME * r4aCameraFindFrameSize(R4A_FRAME_SIZE_t frameSize);

// Lookup the pixel format
// Inputs:
//   pixelFormat: One of the R4A_PIXEL_FORMAT_* values
// Outputs:
//   Returns the address of a R4A_CAMERA_PIXEL data structure if found or
//   nullptr when not found
const R4A_CAMERA_PIXEL * r4aCameraFindPixelFormat(R4A_PIXEL_FORMAT_t pixelFormat);

// Verify the enum values against the corresponding tables
void r4aCameraVerifyTables();

//****************************************
// Generic I2C Class
//****************************************

// I2C device description
typedef struct _R4A_I2C_DEVICE_DESCRIPTION
{
    R4A_I2C_ADDRESS_t i2cAddress; // I2C device address
    const char * displayName;   // Name to display when the device is found
} R4A_I2C_DEVICE_DESCRIPTION;

typedef struct _R4A_I2C_BUS
{
    // Private
    const R4A_I2C_DEVICE_DESCRIPTION * const _deviceTable; // I2C device table
    const int _deviceTableEntries; // Number of entries in the I2C device table
    uint8_t _present[R4A_I2C_ADDRESSES / 8]; // Device detected on the I2C bus during enumeration
    bool _enumerated;       // Has the bus been sucessfully enumerated?
} R4A_I2C_BUS;

extern R4A_I2C_BUS * r4aI2cBus; // I2C bus used in menus

// Enumerate the I2C bus
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   display: Device used for output
void r4aI2cBusEnumerate(R4A_I2C_BUS * i2cBus,
                        Print * display = &Serial);

// Enumerate the 10-bit I2C bus
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   display: Device used for output
void r4aI2cBusEnumerate10Bit(R4A_I2C_BUS * i2cBus,
                             Print * display = &Serial);

// Ping an I2C device to see if it responds
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   i2cAddress: Device address on the I2C bus
// Outputs:
//   Returns true if the device is present and false if NACK received
bool r4aI2cBusEnumerateDevice(R4A_I2C_BUS * i2cBus,
                              R4A_I2C_ADDRESS_t i2cAddress);

// Check if an I2C device was seen during the enumeration
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   i2cAddress: Device address on the I2C bus
// Outputs:
//   Returns true if device detected, false otherwise
bool r4aI2cBusIsDevicePresent(R4A_I2C_BUS * i2cBus,
                              R4A_I2C_ADDRESS_t i2cAddress);

// Read data from an I2C peripheral
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   i2cAddress: Device address on the I2C bus
//   readBuffer: Address of the buffer to receive the data bytes, may be nullptr
//   readByteCount: Size in bytes of the data buffer, maximum receive bytes
//   bytesReadAddr: Address to receive the number of bytes read, may be nullptr
//   display: Device used for debug output
// Outputs:
//   Returns true if successful and false upon failure
bool r4aI2cBusRead(R4A_I2C_BUS * i2cBus,
                   R4A_I2C_ADDRESS_t i2cAddress,
                   uint8_t * readBuffer,
                   size_t readByteCount,
                   size_t * bytesReadAddr = nullptr,
                   Print * display = nullptr);

// Send data to an I2C peripheral
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   i2cAddress: Device address on the I2C bus
//   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
//   dataByteCount: Number of bytes to send from the data buffer
//   display: Device used for debug output
// Outputs:
//   Returns true upon success, false otherwise
bool r4aI2cBusWrite(R4A_I2C_BUS * i2cBus,
                    R4A_I2C_ADDRESS_t i2cAddress,
                    const uint8_t * dataBuffer,
                    size_t dataByteCount,
                    Print * display = nullptr);

// Send data, restart, read data from an I2C peripheral
// Inputs:
//   i2cBus: Address of a R4A_I2C_BUS data structure
//   i2cAddress: Device address on the I2C bus
//   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
//   dataByteCount: Number of bytes to send from the data buffer
//   readBuffer: Address of the buffer to receive the data bytes, may be nullptr
//   readByteCount: Size in bytes of the data buffer, maximum receive bytes
//   bytesReadAddr: Address to receive the number of bytes read from the I2C device,
//                  may be nullptr
//   display: Device used for debug output
// Outputs:
//   Return true upon success and false upon failure
bool r4aI2cBusWriteRead(R4A_I2C_BUS * i2cBus,
                        R4A_I2C_ADDRESS_t i2cAddress,
                        const uint8_t * dataBuffer,
                        size_t dataByteCount,
                        uint8_t * readBuffer,
                        size_t readByteCount,
                        size_t * bytesReadAddr = nullptr,
                        Print * display = nullptr);

//****************************************
// I2C menu API
//****************************************

extern const R4A_MENU_ENTRY r4aI2cMenuTable[]; // I2C menu
#define R4A_I2C_MENU_ENTRIES        4   // Number of menu entries

// Enumerate the I2C bus
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aI2cMenuEnumerate(const R4A_MENU_ENTRY * menuEntry,
                         const char * command,
                         Print * display);

// Read data from the I2C device
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aI2cMenuRead(const R4A_MENU_ENTRY * menuEntry,
                    const char * command,
                    Print * display);

// Write data to the I2C device
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aI2cMenuWrite(const R4A_MENU_ENTRY * menuEntry,
                     const char * command,
                     Print * display);

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

    uint16_t _channelModified;          // One bit per channel, bit set if modified
    uint8_t  _channelRegs[R4A_PCA9685_CHANNEL_COUNT << 2];  // Copy of channel registers
    uint32_t _clockHz;                  // Operating frequence
    const uint32_t _externalClockHz;    // External clock frequency
    R4A_I2C_BUS * const _i2cBus;        // I2C bus to access the PCA9586
    const R4A_I2C_ADDRESS_t  _i2cAddress;     // Address of the PCA9586
    uint16_t _max[R4A_PCA9685_CHANNEL_COUNT]; // Maximum value for this channel
    uint16_t _min[R4A_PCA9685_CHANNEL_COUNT]; // Minimum value for this channel
    size_t _writeBufferLength;          // Length of the write buffer
    uint8_t * _writeBuffer;             // Buffer for I2C write operations

public:

    // Constructor
    // Inputs:
    //   i2cBus: Address of an R4A_I2C object
    //   i2cAddress: Address of the PA9685 on the I2C bus
    //   scanClockHertz: Approximate frequency to scan the LEDs (23 - 1525)
    //   externalClockHertz: Frequency of external clock, zero (0) for internal clock
    R4A_PCA9685(R4A_I2C_BUS * i2cBus,
                R4A_I2C_ADDRESS_t i2cAddress,
                uint32_t scanClockHertz,
                uint32_t externalClockHertz = 25 * 1000 * 1000);

    // Destructor
    ~R4A_PCA9685();

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
                        int16_t onTime,
                        Print * display = nullptr);

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
                             Print * display = nullptr);

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

    // Get the maximum value
    // Inputs:
    //   channel: Channel number (0 - 15)
    // Outputs:
    //   Returns the maximum value for the channel
    int16_t getMaximum(uint8_t channel);

    // Get the minimum value
    // Inputs:
    //   channel: Channel number (0 - 15)
    // Outputs:
    //   Returns the minimum value for the channel
    int16_t getMinimum(uint8_t channel);

    // Set the LED on and off times
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   onTime: Value between 0 and 4096, Time = value / (4096 * PA9685 frequency Hz)
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful and false otherwise
    bool ledOnOff(uint8_t channel,
                  int16_t onTime,
                  Print * display = nullptr);

    // Read one or more PCA9685 registers
    // Inputs:
    //   firstRegisterAddress: Address of the first PA9685 register to be read
    //   dataBuffer: Address of the buffer to receive the register values
    //   dataByteCount: Number of bytes to read from the PA9685 device
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if the read was successful and false upon failure
    bool readRegisters(uint8_t firstRegisterAddress,
                       uint8_t * dataBuffer,
                       size_t dataByteCount,
                       Print * display = nullptr);

    // Convert from degrees (0 - 180) to onTime for servo positioning
    // Inputs:
    //   degrees:  Servo position in degrees (0 - 180)
    // Outputs:
    //   Return the onTicks to program into the PCA9685
    int16_t servoDegreesToOnTicks(uint8_t degrees);

    // Convert from onTime for servo positioning to degrees (0 - 180)
    // Inputs:
    //   onTicks: Ticks for onTime programmed into the PCA9685
    // Outputs:
    //   Returns the degrees for the servo position
    uint8_t servoOnTicksToDegrees(int16_t onTime);

    // Set the servo position
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   degrees: Value between 0 and 180
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful and false otherwise
    bool servoPosition(uint8_t channel,
                       uint8_t degrees,
                       Print * display = nullptr);

    // Set the minimum and maximum values
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   minimum: Value between 0 and 4096
    //   maximum: Value between 0 and 4096
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful and false otherwise
    bool setMinMax(uint8_t channel,
                   int16_t minimum,
                   int16_t maximum,
                   Print * display = nullptr);

    // Set the minimum and maximum values
    // Inputs:
    //   channel: Channel number (0 - 15)
    //   minimum: Value between 0 and 180
    //   maximum: Value between 0 and 180
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful and false otherwise
    bool setMinMaxDegrees(uint8_t channel,
                          uint8_t minimum,
                          uint8_t maximum,
                          Print * display = nullptr);

    // Write the buffered register data to the PCB9685 registers
    // Inputs:
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful, false otherwise
    bool writeBufferedRegisters(Print * display = nullptr);

  private:

    // Display mode 1 register
    // Inputs:
    //   display: Device used for output
    void displayMode1(Print * display = &Serial);

    // Display mode 2 register
    // Inputs:
    //   display: Device used for output
    void displayMode2(Print * display = &Serial);

    // Write data to the PCB9685 registers
    // Inputs:
    //   firstRegisterAddress: Address of the first PA9685 register to write
    //   dataBuffer: Address of the buffer containing the data to write
    //   dataByteCount: Number of bytes to write to the PA9685 device
    //   display: Device used for debug output, may be nullptr
    // Outputs:
    //   Returns true if successful, false otherwise
    bool writeRegisters(uint8_t firstRegisterAddress,
                        uint8_t * dataBuffer,
                        size_t dataByteCount,
                        Print * display = nullptr);

};

//****************************************
// PCA9685 Motor API
//****************************************

#define R4A_PCA9685_MOTOR_SPEED_MAX     4096

class R4A_PCA9685_MOTOR
{
private:

    const bool    _dualChannel;     // True when using 2 channels to control the motor
    const uint8_t _minusChannel;    // Minus channel when using 2 channels
    R4A_PCA9685 * const _pca9685;   // R4A_PCA9684 object address
    const uint8_t _plusChannel;     // Plus channel for 2 channels, channel for 1 channel

public:
    // Constructor
    // Inputs:
    //   R4A_PCA9685 * pca9685: Address of the R4A_PCA9684 object
    //   channel: Channel controlling the motor
    R4A_PCA9685_MOTOR(R4A_PCA9685 * pca9685, uint8_t channel);

    // Constructor
    // Inputs:
    //   R4A_PCA9685 * pca9685: Address of the R4A_PCA9684 object
    //   plusChannel: Channel that is positive during forward speeds
    //   minusChannel: Channel that is grounded during forward speeds
    R4A_PCA9685_MOTOR(R4A_PCA9685 * pca9685,
                      uint8_t plusChannel,
                      uint8_t minusChannel);

    // Destructor
    ~R4A_PCA9685_MOTOR();

    // Buffers the brake value, call write to apply the brakes.  Applies
    // plus to both channels.  When the brakes are not applied the motor
    // is coasting.
    // Inputs:
    //   brakeValue: Value in the range of no braking (0) to full braking (4096)
    //   display: Object used to display debug messages for this transaction or nullptr
    // Outputs:
    //   Returns true if successful, always returns false for single channel
    //   operation.
    bool brake(int16_t brakeValue, Print * display = nullptr);

    // Buffers the coasting value, call PCA9685.wrie to start coasting.
    // Removes power from both sides of the motor.
    // Inputs:
    //   display: Object used to display debug messages for this transaction or nullptr
    // Outputs:
    //   Returns true if successful.
    bool coast(Print * display = nullptr);

    // Display the motor state
    // Inputs:
    //   display: Object used to display debug messages or nullptr
    void display(Print * display);

    // Get the minus channel associated with this motor
    // Outputs:
    //   Returns the minus channel number
    uint8_t getMinusChannel();

    // Get the plus channel associated with this motor
    // Outputs:
    //   Returns the plus channel number
    uint8_t getPlusChannel();

    // Buffers the motor speed value, call PCA9685.write to apply the speed
    // values.  Positive speed values applies positive voltage to the plus
    // channel and connects the minus channel to ground.  Negative speed
    // values applies positive voltage to the minus channel and connects
    // the plus channel to ground.
    // Inputs:
    //   speedValue: Value in the range of full reverse (-4096) to full forward (4096),
    //               where (0) is coasting
    //   display: Object used to display debug messages for this transaction or nullptr
    // Outputs:
    //   Returns true if successful.
    bool speed(int16_t speedValue, Print * display = nullptr);

    // Writes all of buffered values to the PCA9685.
    // Inputs:
    //   display: Object used to display debug messages for this transaction or nullptr
    // Outputs:
    //   Returns true if successful, false otherwise.
    bool write(Print * display = nullptr);
};

extern R4A_PCA9685_MOTOR * const r4aPca9685MotorTable[]; // Table of motors
extern const int r4aPca9685MotorTableEntries; // Number of motors in the table

// Brake the motor
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Inputs:
//   motor: Zero based index into r4aPca9685MotorTable
//   speed: Speed of the motor, range: 0 to 4096
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorBrake(uint8_t motor, int16_t speed = 4096, Print * display = nullptr);

// Brake all the motors
// Start bit, I2C device address, ACK, register address, ACK, 32 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 770 uSec = (1+8+1+8+1+((8+1)×32)+1)÷(400×1000)
// Inputs:
//   speed: Speed of the motor, range: 0 to 4096
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorBrakeAll(int16_t speed = 4096, Print * display = nullptr);

// Place the motor in the coasting state
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Inputs:
//   motor: Zero based index into r4aPca9685MotorTable
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorCoast(uint8_t motor, Print * display = nullptr);

// Place the motors in the coasting state
// Start bit, I2C device address, ACK, register address, ACK, 4 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 410 uSec = (1+8+1+8+1+((8+1)×16)+1)÷(400×1000)
// Inputs:
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorCoastAll(Print * display = nullptr);

// Display the motor state
// Inputs:
//   display: Object address used to display debug messages or nullptr
void r4aPca9685MotorDisplayState(Print * display = &Serial);

// Drive the motor forward
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Inputs:
//   motor: Zero based index into r4aPca9685MotorTable
//   speed: Speed of the motor, range: 0 to 4096
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorForward(uint8_t motor, int16_t speed, Print * display = nullptr);

// Drive the motor reverse
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Inputs:
//   motor: Zero based index into r4aPca9685MotorTable
//   speed: Speed of the motor, range: 0 to 4096
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorReverse(uint8_t motor, int16_t speed, Print * display = nullptr);

// Set the motor speed
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Inputs:
//   motor: Zero based index into r4aPca9685MotorTable
//   speed: Speed of the motor, range: -4096 to 4096
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorSetSpeed(uint8_t motor, int16_t speed, Print * display = nullptr);

// Initialize the motors
// Inputs:
//   display: Object address used to display debug messages or nullptr
// Outputs:
//   Returns true if successful, false upon failure
bool r4aPca9685MotorSetup(Print * display = nullptr);

//****************************************
// PCA9685 Motor Menu API
//****************************************

#define R4A_PCA9685_MOTOR_MENU_ENTRIES      7
extern const R4A_MENU_ENTRY r4aPca9685MotorMenuTable[];

// Brake the motors
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuBrake(const R4A_MENU_ENTRY * menuEntry,
                              const char * command,
                              Print * display = nullptr);

// Coast the motors
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuCoast(const R4A_MENU_ENTRY * menuEntry,
                              const char * command,
                              Print * display = nullptr);

// Display the motors
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuDisplay(const R4A_MENU_ENTRY * menuEntry,
                          const char * command,
                          Print * display = nullptr);

// Drive the motors forward
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuForward(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display = nullptr);

// Display the help text with mm
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aPca9685MotorMenuHelpMm(const struct _R4A_MENU_ENTRY * menuEntry,
                               const char * align,
                               Print * display = &Serial);

// Display the help text with mm and ssss
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aPca9685MotorMenuHelpMmSsss(const struct _R4A_MENU_ENTRY * menuEntry,
                                   const char * align,
                                   Print * display = &Serial);

// Drive the motors in reverse
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuReverse(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display = nullptr);

// Stop the motors
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685MotorMenuStop(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display = nullptr);

//****************************************
// PCA9685 Servo API
//****************************************

class R4A_PCA9685_SERVO
{
private:

    R4A_PCA9685 * const _pca9685;   // R4A_PCA9684 object address
    const uint8_t _channel;         // Channel for the servo

public:

    const uint8_t _maximum;         // Degrees maximum
    const uint8_t _minimum;         // Degrees minimum

    // Constructor
    // Inputs:
    //   R4A_PCA9685 * pca9685: Address of the R4A_PCA9684 object
    //   channel: Channel controlling the servo
    //   minimum: Stop point for the servo specified in degrees, range (0 - 180)
    //   maximum: Stop point for the servo specified in degrees, range (0 - 180)
    R4A_PCA9685_SERVO(R4A_PCA9685 * pca9685,
                      uint8_t channel,
                      uint8_t minimum = 0,
                      uint8_t maximum = 180);

    // Destructor
    ~R4A_PCA9685_SERVO();

    // Get the position of the servo
    // Inputs:
    //   display: Object used to display debug messages or nullptr
    // Outputs:
    //   Returns the position of the servo
    uint8_t positionGet(Print * display = nullptr);

    // Get the maximum position of the servo
    // Outputs:
    //   Returns the maximum position of the servo
    uint8_t positionMax();

    // Get the minimum position of the servo
    // Outputs:
    //   Returns the minimum position of the servo
    uint8_t positionMin();

    // Set the position of the servo
    // Inputs:
    //   degrees: Angle to rotate the servo, default range 0 - 180
    //   display: Object used to display debug messages or nullptr
    // Outputs:
    //   Returns true if successful and false otherwise
    bool positionSet(uint8_t position, Print * display = nullptr);
};

//****************************************
// PCA9685 Servo Menu API
//****************************************

extern R4A_PCA9685_SERVO * const r4aPca9685ServoTable[]; // Table of servos
extern const int r4aPca9685ServoTableEntries; // Number of servos in the table

#define R4A_PCA9685_SERVO_MENU_ENTRIES      5
extern const R4A_MENU_ENTRY r4aPca9685ServoMenuTable[];

// Display the help text with ddd
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
void r4aPca9685ServoMenuHelpDdd(const struct _R4A_MENU_ENTRY * menuEntry,
                                const char * align,
                                Print * display);

// Display the help text with s and ddd
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
void r4aPca9685ServoMenuHelpSDdd(const struct _R4A_MENU_ENTRY * menuEntry,
                                 const char * align,
                                 Print * display);

// Display the servo states
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685ServoMenuDisplay(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display = nullptr);

// Set the servo specified by the menu parameter to ddd degrees
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685ServoMenuMove(const R4A_MENU_ENTRY * menuEntry,
                             const char * command,
                             Print * display = nullptr);

// Set servo s to ddd degrees
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aPca9685ServoMenuSet(const R4A_MENU_ENTRY * menuEntry,
                            const char * command,
                            Print * display = nullptr);

//****************************************
// PCF8574 API
//****************************************

class R4A_PCF8574
{
private:

    R4A_I2C_BUS * const _i2cBus;    // I2C bus to access the PCF8574
    const R4A_I2C_ADDRESS_t  _i2cAddress; // Address of the PCF8574

public:

    // Constructor
    // Inputs:
    //   i2cBus: Address of an R4A_I2C object
    //   i2cAddress: Address of the PA9685 on the I2C bus
    R4A_PCF8574(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress);

    // Destructor
    ~R4A_PCF8574();

    // Read data from the PCF8574 port
    // Inputs:
    //   data: Address of buffer to receive the data byte
    // Outputs:
    //   Returns true if the data byte was read successfully and false otherwise
    bool read(uint8_t *data);

    // Write data to the PCF8574 port
    // Inputs:
    //   data: Byte of data to be written
    // Outputs:
    //   Returns true if the data byte was successfully written and false
    //   otherwise
    bool write(uint8_t data);
};

//****************************************
// SX1509
//****************************************

typedef struct _R4A_SX1509
{
    R4A_I2C_BUS * _i2cBus;
    R4A_I2C_ADDRESS_t _i2cAddress;
} R4A_SX1509;

// Display the SX1509 registers
// Inputs:
//   sx1509: Address of a R4A_SX1509 data structure
//   display: Device used for output
void r4aSx1509DisplayRegisters(R4A_SX1509 * sx1509,
                               Print * display);

// Display a SX1509 register
// Inputs:
//   sx1509: Address of a R4A_SX1509 data structure
//   registerAddress: Register number to display
//   registerName: Address of a zero terminated name string
//   display: Device used for output
void r4aSx1509RegisterDisplay(R4A_SX1509 * sx1509,
                              uint8_t registerAddress,
                              const char * registerName,
                              Print * display = &Serial);

// Modify a register's value
//      Clear bit: Bit in andMask is zero, bit in xorMask is zero
//      Set bit: Bit in andMask is zero, bit in xorMask is one
//      No change: Bit in andMask is one, bit in xorMask is zero
//      Toggle bit: Bit in andMask is one, bit in xorMask is one
// Inputs:
//   registerAddress: Address of the SX1509 register
//   andMask: Bits keep from the register value
//   xorMask: Bits to toggle in the register value
// Outputs:
//   Returns true upon success and false on failure
bool r4aSx1509RegisterModify(R4A_SX1509 * sx1509,
                             uint8_t registerAddress,
                             uint8_t andMask,
                             uint8_t xorMask,
                             Print *display = nullptr);

// Read a SX1509 register
// Inputs:
//   sx1509: Address of a R4A_SX1509 data structure
//   registerAddress: Register number to read
//   data: Address to receive the register value
//   display: Device used for error output, may be nullptr
// Outputs:
//   Returns true up success and false upon failure
bool r4aSx1509RegisterRead(R4A_SX1509 * sx1509,
                           uint8_t registerAddress,
                           uint8_t * data,
                           Print * display = nullptr);

// Write a SX1509 register
// Inputs:
//   sx1509: Address of a R4A_SX1509 data structure
//   registerAddress: Register number to write
//   data: Value to write to the register
//   display: Device used for error output, may be nullptr
// Outputs:
//   Returns true up success and false upon failure
bool r4aSx1509RegisterWrite(R4A_SX1509 * sx1509,
                            uint8_t registerAddress,
                            uint8_t data,
                            Print * display);

//****************************************
// VK16K33
//****************************************

// LED Matrix (VK16K33) menu
extern const R4A_MENU_ENTRY r4aVk16k33MenuTable[];
#define R4A_VK16K33_MENU_ENTRIES  8

#define R4A_VK16K33_MAX_COLUMNS         16
#define R4A_VK16K33_MAX_ROWS            8
#define R4A_VK16K33_PIXEL_OFFSET        1   // Leave room for the command

typedef struct _R4A_VK16K33
{
    R4A_I2C_BUS * i2cBus;
    R4A_I2C_ADDRESS_t i2cAddress;
    const uint8_t * const columnMap;
    const uint8_t * const rowMap;
    uint8_t columns;
    uint8_t rows;
    uint8_t brightness;
    uint8_t pixels[R4A_VK16K33_PIXEL_OFFSET + R4A_VK16K33_MAX_COLUMNS];
} R4A_VK16K33;

// Fonts
extern const uint8_t * r4a5x7Numbers[];
extern const uint8_t * r4a5x7UcAtoI[];
extern const uint8_t * r4a5x7LcAtoH[];
extern const uint8_t r4a5x7Font_asterisk[5];
extern const uint8_t r4a5x7Font_dash[5];
extern const uint8_t r4a5x7Font_dp;
extern const uint8_t r4a5x7Font_l;
extern const uint8_t r4a5x7Font_t[5];

// Set the brightness (0-15)
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   brightness: A value from 0 to 15
//   display: Device used for output
// Outputs:
//   Returns true if successful and false upon failure
bool r4aVk16k33Brightness(R4A_VK16K33 * vk16k33,
                          uint8_t brightness,
                          Print * display = nullptr);

// Clear the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
void r4aVk16k33BufferClear(R4A_VK16K33 * vk16k33);

// Fill the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   data: Value to place in each of the columns
void r4aVk16k33BufferFill(R4A_VK16K33 * vk16k33, uint8_t data);

// Display a character on the LED matrix
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   xColumn: Column number to modify, zero on left
//   data: Character to write into the column
void r4aVk16k33DisplayChar(R4A_VK16K33 * vk16k33, int xColumn, char data);

// Display "Halt" on the LED matrix
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
void r4aVk16k33DisplayHalt(R4A_VK16K33 * vk16k33);

// Display "Idle" on the LED matrix
void r4aVk16k33DisplayIdle(R4A_VK16K33 * vk16k33);

// Turn on the display, start the scanning of the LEDs
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   display: Device used for output
// Outputs:
//   Returns true if successful and false upon failure
bool r4aVk16k33DisplayOn(R4A_VK16K33 * vk16k33, Print * display = nullptr);

// Copy the RAM buffer to the display
// Start bit, I2C device address, ACK, register address, ACK, 16 data bytes
// with ACKs and a stop bit, all at 400 KHz
// ~410 uSec = (1+8+1+8+1+((8+1)×16)+1)÷(400×1000)
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   display: Device used for output
// Outputs:
//   Returns true if successful and false upon failure
bool r4aVk16k33DisplayPixels(R4A_VK16K33 * vk16k33, Print * display = nullptr);

// Turn off all pixels in the LED matrix
void r4aVk16k33MenuClear(const R4A_MENU_ENTRY * menuEntry,
                         const char * command,
                         Print * display);

// Display the date on the LED matrix
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aVk16k33MenuDate(const R4A_MENU_ENTRY * menuEntry,
                        const char * command,
                        Print * display);

// Turn on all pixels in the LED matrix
void r4aVk16k33MenuFill(const R4A_MENU_ENTRY * menuEntry,
                        const char * command,
                        Print * display);

// Display Halt on the LED matrix
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aVk16k33MenuHalt(const R4A_MENU_ENTRY * menuEntry,
                        const char * command,
                        Print * display);

// Display Idle on the LED matrix
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aVk16k33MenuIdle(const R4A_MENU_ENTRY * menuEntry,
                        const char * command,
                        Print * display);

// Display the current time on the LED matrix
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aVk16k33MenuTime(const R4A_MENU_ENTRY * menuEntry,
                        const char * command,
                        Print * display);

// Display up to 3 characters on the display
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   command: Zero terminated command string
//   display: Device used for output
void r4aVk16k33MenuWrite(const R4A_MENU_ENTRY * menuEntry,
                         const char * command,
                         Print * display);

// Turn on the VK16K33 LED controller
// Clear a pixel in the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   display: Device used for output
// Outputs:
//   Returns true if the pixel was cleared in the RAM buffer
bool r4aVk16k33On(R4A_VK16K33 * vk16k33, Print * display = nullptr);

// Clear a pixel in the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   column: Number of the column to modify
//   row: Number of the row to modify
// Outputs:
//   Returns true if the pixel was cleared in the RAM buffer
bool r4aVk16k33PixelClear(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t row);

// Set a pixel in the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   column: Number of the column to modify
//   row: Number of the row to modify
// Outputs:
//   Returns true if the pixel was set in the RAM buffer
bool r4aVk16k33PixelSet(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t row);

// Initialize the VK16K33
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   display: Device used for output
bool r4aVk16k33Setup(R4A_VK16K33 * vk16k33, Print * display = nullptr);

// Write a column of eight pixels in the RAM buffer
// Inputs:
//   vk16k33: Address of a R4A_VK16K33 data structure
//   column: Column number to update with the data
//   data: Data for the 8-bit column
bool r4aVk16k33WriteColumn(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t data);

//****************************************
// u-blox ZED F9P class
//****************************************

// Parameters
extern bool r4aZedF9pDisplayAltitude;   // Display the altitude
extern bool r4aZedF9pDisplayAltitudeStdDev; // Display the altitude standard deviation
extern bool r4aZedF9pDisplayFixType;    // Display the fix type
extern bool r4aZedF9pDisplayHorizAcc;   // Display the horizontal accuracy
extern bool r4aZedF9pDisplayHorizAccStdDev; // Display the hroizontal accuracy standard deviation
extern bool r4aZedF9pDisplayLatitude;   // Display the latitude
extern bool r4aZedF9pDisplayLatitudeStdDev;  // Display the latitude standard deviation
extern bool r4aZedF9pDisplayLongitude;  // Display the longitude
extern bool r4aZedF9pDisplayLongitudeStdDev; // Display the longitude standard deviation
extern bool r4aZedF9pDisplaySiv;        // Display satellites-in-view
extern bool r4aZedF9pDisplayTime;       // Display time data

extern uint32_t r4aZedF9pLocationDisplayMsec; // 0 = Off, Interval to display the location
extern uint32_t r4aZedF9pPollMsec;      // I2C polling interval for the GNSS receiver
extern bool r4aZedF9pUnitsFeetInches;   // Display in feet and inches .vs. meters

// Routine to display the computed point
// Inputs:
//   parameter: Callback parameter passed to computePoint
//   comment: Text to display at the start of the line
//   latitude: Latitude in degrees
//   latitudeStdDev: Latitude standard deviation in degrees
//   longitude: Longitude in degrees
//   longitudeStdDev: Longitude standard deviation in degrees
//   altitude: Altitude in meters
//   altitudeStdDev: Altitude standard deviation in meters
//   horizontalAccuracy: Accuracy in meters
//   horizontalAccuracyStdDev: Horizontal accuracy standard deviation in meters
//   satellitesInView: The number of satellites feeding the GNSS receiver
//   display: Device used for output, passed to computePoint
typedef void (* R4A_DISPLAY_ROUTINE)(intptr_t parameter,
                                     const char * comment,
                                     double latitude,
                                     double latitudeStdDev,
                                     double longitude,
                                     double longitudeStdDev,
                                     double altitude,
                                     double altitudeStdDev,
                                     double horizontalAccuracy,
                                     double horizontalAccuracyStdDev,
                                     uint8_t satellitesInView,
                                     Print * display);

// Define the I2C driver for UBLOX GNSS devices
class R4A_UBLOX_I2C : public SparkFun_UBLOX_GNSS::GNSSDeviceBus
{
private:
    R4A_I2C_BUS *_i2cBus;
    R4A_I2C_ADDRESS_t _i2cAddress;

public:
    // Constructor
    // Inputs:
    //   i2cBus: Address of an initialized R4A_I2C_BUS object
    //   i2cAddress: Address of the GNSS device on the I2C bus
    R4A_UBLOX_I2C(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress);

    // Checks how many bytes are waiting in the GNSS's I2C buffer
    // Outputs:
    //   Number of bytes in the GNSS device waiting to be read
    uint16_t available();

    // Is the GNSS device connected to the I2C bus?
    bool ping();

    // Read data from the GNSS device
    // Inputs:
    //   data: Address of the buffer to receive the data
    //   length: Number of bytes to read
    // Outputs:
    //   Returns the number of bytes read or zero upon failure
    uint8_t readBytes(uint8_t *data, uint8_t length);

    // Read data from the GNSS device
    // Inputs:
    //   data: Address of the buffer containing the data to write
    //   length: Number of bytes to write
    // Outputs:
    //   Returns the number of bytes written or zero upon failure
    uint8_t writeBytes(uint8_t *data, uint8_t length);

    // Unused functions required by the GNSSDeviceBus class
    void startWriteReadByte();
    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length);
    void writeReadByte(const uint8_t *data, uint8_t *readData);
    void writeReadByte(const uint8_t data, uint8_t *readData);
    void endWriteReadByte();
};

// Connect the UBLOX GNSS device to the UBLOX I2C driver
class R4A_UBLOX_I2C_GNSS : public DevUBLOXGNSS
{
private:
    // I2C bus class
    R4A_UBLOX_I2C _ubloxI2cBus;

public:
    // Constructor
    // Inputs:
    //   i2cBus: Address of an initialized R4A_I2C_BUS object
    //   i2cAddress: Address of the GNSS device on the I2C bus
    R4A_UBLOX_I2C_GNSS(R4A_I2C_BUS * i2cBus,
                       R4A_I2C_ADDRESS_t i2cAddress = kUBLOXGNSSDefaultAddress);

    // This method is called to initialize the SFE_UBLOX_GNSS library.  This
    // method must be called before calling any other method that interacts
    // with the device.
    // Inputs:
    //   maxWait:
    //   assumeSuccess:
    // Outputs:
    //   Returns true if successful and false upon failure
    bool begin(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait,
               bool assumeSuccess = false);
};

class R4A_ZED_F9P
{
  private:

    double _altitudeMean;
    double _altitudeStdDev;
    const char * _comment;
    Print * _display;
    R4A_UBLOX_I2C_GNSS _gnss;
    double _horizontalMean;
    double _horizontalStdDev;
    const R4A_I2C_ADDRESS_t _i2cAddress;
    double _latitudeMean;
    double _latitudeStdDev;
    double _longitudeMean;
    double _longitudeStdDev;

    // Start collecting data for a point
    // Inputs:
    //   count: Number of points to average
    //   comment: Text to display at the start of the line
    //   display: Device used for output
    bool collectData(int count, const char * comment, Print * display = &Serial);

  public:

    double _altitude;
    double * _altitudeArray;
    int _altitudeCount;
    int _altitudeCountSave;
    uint8_t _carrierSolution;
    bool _confirmedDate;
    bool _confirmedTime;
    uint8_t _day;
    R4A_DISPLAY_ROUTINE _displayRoutine;
    intptr_t _displayParameter;
    uint8_t _fixType;
    bool _fullyResolved;
    double _horizontalAccuracy;
    double * _horizontalAccuracyArray;
    uint8_t _hour;
    volatile bool _hpDataAvailable;
    const uint8_t _i2cTransactionSize;
    double _latitude;
    double * _latitudeArray;
    int _latLongCount;
    int _latLongCountSave;
    double _longitude;
    double * _longitudeArray;
    uint16_t _millisecond; // Limited to first two digits
    uint8_t _minute;
    uint8_t _month;
    int32_t _nanosecond;
    bool _online;   // True if ZED F9P successfully initialized
    uint8_t _satellitesInView;
    uint8_t _second;
    uint32_t _tAcc;
    bool _unitsFeetInches;
    bool _validDate;
    bool _validTime;
    uint16_t _year;

    // Constructor
    R4A_ZED_F9P(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress);

    // Destructor
    ~R4A_ZED_F9P();

    // Initialize the GNSS device
    bool begin(Print * display = &Serial);

    // Compute the mean and standard deviation
    // Inputs:
    //   data: Address of an array of data
    //   entries: Number of entries in the array
    //   standardDeviation: Address of a location to return the standard
    //                      deviation value
    // Outputs:
    //   Return the mean of the values in the array
    double computeMean(double * data,
                       int entries,
                       double * standardDeviation);

    // Compute and display a point
    // Inputs:
    //   routine: Callback routine once point is computed
    //   parameter: Parameter for the callback routine
    //   count: Number of points to average
    //   comment: Text to display at the start of the line
    //   display: Device used for output
    void computePoint(R4A_DISPLAY_ROUTINE routine,
                      intptr_t parameter,
                      int count,
                      const char * comment,
                      Print * display = &Serial);

    // Display the location
    // Inputs:
    //   comment: Text to display at the start of the line
    //   display: Device used for output
    void displayLocation(const char * comment,
                         Print * display = &Serial);

    // Display the location
    // Inputs:
    //   comment: Text to display at the start of the line
    //   unitsFeetInches: True to display in feet and inches instead of meters
    //   displayTime: True to display the time
    //   displaySiv: True to display the satellites-in-view count
    //   displayLatitude: True to display the latitude
    //   displayLongitude: True to display the longitude
    //   displayHorizAcc: True to display the horizontal accuracy
    //   displayAltitude: True to display the altitude
    //   displayFixType: True to display the fix type
    //   display: Device used for output
    void displayLocation(const char * comment,
                         bool unitsFeetInches,
                         bool displayTime,
                         bool displaySiv,
                         bool displayLatitude,
                         bool displayLongitude,
                         bool displayHorizAcc,
                         bool displayAltitude,
                         bool displayFixType,
                         Print * display = &Serial);

    // Display the location
    // Inputs:
    //   comment: Text to display at the start of the line
    //   latitude: Latitude in degrees
    //   longitude: Longitude in degrees
    //   altitude: Altitude in meters
    //   horizontalAccuracy: Horizontal accuracy in meters
    //   satellitesInView: The number of satellites feeding the GNSS receiver
    //   display: Device used for output
    void displayLocation(const char * comment,
                         double latitude,
                         double longitude,
                         double altitude,
                         double horizontalAccuracy,
                         uint8_t satellitesInView,
                         Print * display = &Serial);

    // Display the location
    // Inputs:
    //   comment: Text to display at the start of the line
    //   latitude: Latitude in degrees
    //   latitudeStdDev: Latitude standard deviation in degrees
    //   longitude: Longitude in degrees
    //   longitudeStdDev: Longitude standard deviation in degrees
    //   altitude: Altitude in meters
    //   altitudeStdDev: Altitude standard deviation in meters
    //   horizontalAccuracy: Horizontal accuracy in meters
    //   horizontalAccuracyStdDev: Horizontal accuracy standard deviation in meters
    //   satellitesInView: The number of satellites feeding the GNSS receiver
    //   unitsFeetInches: True to display in feet and inches instead of meters
    //   displayTime: True to display the time
    //   displaySiv: True to display the satellites-in-view count
    //   displayLatitude: True to display the latitude
    //   displayLatStdDev: True to display the latitude standard deviation
    //   displayLongitude: True to display the longitude
    //   displayLongStdDev: True to display the longitude standard deviation
    //   displayHorizAcc: True to display the horizontal accuracy
    //   displayHorizAccStdDev: True to display the horizontal accuracy standard deviation
    //   displayAltitude: True to display the altitude
    //   displayAltitudeStdDev: True to display the altitude standard deviation
    //   displayFixType: True to display the fix type
    //   display: Device used for output
    void displayLocation(const char * comment,
                         double latitude,
                         double latitudeStdDev,
                         double longitude,
                         double longitudeStdDev,
                         double altitude,
                         double altitudeStdDev,
                         double horizontalAccuracy,
                         double horizontalAccuracyStdDev,
                         uint8_t satellitesInView,
                         bool unitsFeetInches,
                         bool displayTime,
                         bool displaySiv,
                         bool displayLatitude,
                         bool displayLatStdDev,
                         bool displayLongitude,
                         bool displayLongStdDev,
                         bool displayHorizAcc,
                         bool displayHorizAccStdDev,
                         bool displayAltitude,
                         bool displayAltitudeStdDev,
                         bool displayFixType,
                         Print * display = &Serial);

    // Poll the GNSS using I2C
    void i2cPoll();

    // Push the RTCM data to the GNSS using I2C
    // Inputs:
    //   buffer: Address of data to send to the GNSS receiver
    //   bytes: Number of bytes to send to the GNSS receiver
    int pushRawData(uint8_t * buffer, int bytes);

    // Store horizontal position data
    // Inputs:
    //   ubxDataStruct: Address of the data structure containing the high
    //                  precision location data
    void storeHPdata(UBX_NAV_HPPOSLLH_data_t * ubxDataStruct);

    // Store vertical position and time data
    // Inputs:
    //   ubxDataStruct: Address of the data structure containing the
    //                  position, velocity and time values
    void storePVTdata(UBX_NAV_PVT_data_t * ubxDataStruct);

    // Process the received NMEA messages
    // Inputs:
    //   currentMsec: Number of milliseconds since boot
    //   comment: Text to display at the start of the line
    //   display: Device used for output
    void update(uint32_t currentMsec, const char * comment, Print * display = nullptr);
};

// Store horizontal position data
// Inputs:
//   ubxDataStruct: Data structure containing high precision data
void r4aZedF9pStoreHPdata(UBX_NAV_HPPOSLLH_data_t * ubxDataStruct);

// Store vertical position and time data
// Inputs:
//   ubxDataStruct: Data structure containing position, velocity and time
void r4aZedF9pStorePVTdata(UBX_NAV_PVT_data_t * ubxDataStruct);

extern R4A_ZED_F9P * r4aZedF9p;

#endif  // __R4A_I2C_H__
