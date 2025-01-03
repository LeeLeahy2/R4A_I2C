/**********************************************************************
  R4A_I2C.h

  Robots-For-All (R4A)
  Declare the I2C support
**********************************************************************/

#ifndef __R4A_I2C_H__
#define __R4A_I2C_H__

#include <Arduino.h>            // Built-in
#include <Wire.h>               // Built-in

#include <R4A_Robot.h>          // Robots-For-All robot support

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

//****************************************
// Generic I2C Class
//****************************************

// I2C device description
typedef struct _R4A_I2C_DEVICE_DESCRIPTION
{
    uint8_t deviceAddress;      // I2C device address: 0 - 0x7f
    const char * displayName;   // Name to display when the device is found
} R4A_I2C_DEVICE_DESCRIPTION;

// Read data from an I2C peripheral
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   deviceAddress: Device address on the I2C bus (0 - 0x7f)
//   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
//   cmdByteCount: Number of bytes to send from the command buffer
//   dataBuffer: Address of the buffer to receive the data bytes, may be nullptr
//   dataByteCount: Size in bytes of the data buffer, maximum receive bytes
//   display: Device used for debug output
//   releaseI2cBus: A value of true releases the I2C bus after the transaction
// Outputs:
//   Returns the number of bytes read
typedef size_t (* R4A_I2C_BUS_READ)(struct _R4A_I2C_BUS * object,
                                    uint8_t deviceI2cAddress,
                                    const uint8_t * cmdBuffer, // Does not include I2C address
                                    size_t cmdByteCount,
                                    uint8_t * readBuffer,
                                    size_t readByteCount,
                                    Print * display,
                                    bool releaseI2cBus);

// Send data to an I2C peripheral, entered with the I2C bus lock held
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   deviceAddress: Device address on the I2C bus (0 - 0x7f)
//   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
//   cmdByteCount: Number of bytes to send from the command buffer
//   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
//   dataByteCount: Number of bytes to send from the data buffer
//   display: Device used for debug output
//   releaseI2cBus: A value of true releases the I2C bus after the transaction
// Outputs:
//   Returns true upon success, false otherwise
typedef bool (* R4A_I2C_BUS_WRITE_WITH_LOCK)(struct _R4A_I2C_BUS * object,
                                             uint8_t deviceI2cAddress,
                                             const uint8_t * cmdBuffer,
                                             size_t cmdByteCount,
                                             const uint8_t * dataBuffer,
                                             size_t dataByteCount,
                                             Print * display,
                                             bool releaseI2cBus);

typedef struct _R4A_I2C_BUS
{
    // Private
    TwoWire * _i2cBus;      // API for the I2C bus
    const R4A_I2C_DEVICE_DESCRIPTION * const _deviceTable; // I2C device table
    const int _deviceTableEntries; // Number of entries in the I2C device table
    volatile int _lock;     // Lock to synchronize access to the I2C bus
    uint8_t _present[16];   // Device detected on the I2C bus during enumeration

    R4A_I2C_BUS_WRITE_WITH_LOCK _writeWithLock;

    // Public
    R4A_I2C_BUS_READ _read;

    bool _enumerated;       // Has the bus been sucessfully enumerated?
} R4A_I2C_BUS;

extern R4A_I2C_BUS * r4aI2cBus; // I2C bus used in menus

// Enumerate the I2C bus
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   display: Device used for output
void r4aI2cBusEnumerate(R4A_I2C_BUS * object,
                        Print * display = &Serial);

// Ping an I2C device and see if it responds
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   deviceAddress: Device address on the I2C bus (0 - 0x7f)
// Outputs:
//   Returns true if device detected, false otherwise
bool r4aI2cBusEnumerateDevice(R4A_I2C_BUS * object,
                              uint8_t deviceAddress);

// Get the TwoWire pointer
//
// Warning: Using the I2C bus outside of these routines will break the
// I2C controller synchronization leading to hangs, crashes and unspecified
// behavior!
//
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
// Outputs:
//   Returns the TwoWire object address
TwoWire * r4aI2cBusGetTwoWire(R4A_I2C_BUS * object);

// Check if an I2C device was seen during the enumeration
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   deviceAddress: Device address on the I2C bus (0 - 0x7f)
// Outputs:
//   Returns true if device detected, false otherwise
bool r4aI2cBusIsDevicePresent(R4A_I2C_BUS * object,
                              uint8_t deviceAddress);

// Send data to an I2C peripheral
// Inputs:
//   object: Address of a R4A_I2C_BUS data structure
//   deviceAddress: Device address on the I2C bus (0 - 0x7f)
//   cmdBuffer: Address of the buffer containing the command bytes, may be nullptr
//   cmdByteCount: Number of bytes to send from the command buffer
//   dataBuffer: Address of the buffer containing the data bytes, may be nullptr
//   dataByteCount: Number of bytes to send from the data buffer
//   display: Device used for debug output
//   releaseI2cBus: A value of true releases the I2C bus after the transaction
// Outputs:
//   Returns true upon success, false otherwise
bool r4aI2cBusWrite(R4A_I2C_BUS * object,
                    uint8_t deviceI2cAddress,
                    const uint8_t * cmdBuffer,
                    size_t cmdByteCount,
                    const uint8_t * dataBuffer,
                    size_t dataByteCount,
                    Print * display = nullptr,
                    bool releaseI2cBus = true);

//****************************************
// I2C menu API
//****************************************

extern const R4A_MENU_ENTRY r4aI2cMenuTable[]; // I2C menu
#define R4A_I2C_MENU_ENTRIES        4   // Number of menu entries

// Display the help text with mm and ssss
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aI2cMenuHelpAaRr(const struct _R4A_MENU_ENTRY * menuEntry,
                        const char * align,
                        Print * display);

// Display the help text with mm and ssss
// Inputs:
//   menuEntry: Address of the object describing the menu entry
//   align: Zero terminated string of spaces for alignment
//   display: Device used for output
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aI2cMenuHelpAaRrDd(const struct _R4A_MENU_ENTRY * menuEntry,
                          const char * align,
                          Print * display);

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
    const uint8_t  _i2cAddress;         // Address of the PCA9586
    uint16_t _max[R4A_PCA9685_CHANNEL_COUNT]; // Maximum value for this channel
    uint16_t _min[R4A_PCA9685_CHANNEL_COUNT]; // Minimum value for this channel

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
    //   Returns the number of bytes read
    size_t readRegisters(uint8_t firstRegisterAddress,
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
    const uint8_t  _i2cAddress;     // Address of the PCF8574

public:

    // Constructor
    // Inputs:
    //   i2cBus: Address of an R4A_I2C object
    //   i2cAddress: Address of the PA9685 on the I2C bus
    R4A_PCF8574(R4A_I2C_BUS * i2cBus, uint8_t i2cAddress);

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

class R4A_ZED_F9P
{
  private:

    double _altitudeMean;
    double _altitudeStdDev;
    const char * _comment;
    Print * _display;
    SFE_UBLOX_GNSS _gnss;
    double _horizontalMean;
    double _horizontalStdDev;
    const uint8_t _i2cAddress;
    R4A_I2C_BUS * _i2cBus;
    double _latitudeMean;
    double _latitudeStdDev;
    double _longitudeMean;
    double _longitudeStdDev;
    TwoWire * _twoWire;

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
    R4A_ZED_F9P(R4A_I2C_BUS * i2cBus, uint8_t i2cAddress);

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
    //   display: Device used for output
    int pushRawData(uint8_t * buffer, int bytes, Print * display);

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

#endif  // R4A_USING_ESP32
