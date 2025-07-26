/**********************************************************************
  R4A_SX1509.cpp

  Robots-For-All (R4A)
  SX1509 support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Display a SX1509 register
void r4aSx1509RegisterDisplay(R4A_SX1509 * sx1509,
                              uint8_t registerAddress,
                              const char * registerName,
                              Print * display)
{
    uint8_t data;

    // Read the register
    if (r4aSx1509RegisterRead(sx1509,
                              registerAddress,
                              &data,
                              display))
    {
        // Display the register value
        if (display)
            display->printf("0x%02x: %s\r\n", data, registerName);
    }
}

//*********************************************************************
// Modify a register's value
//      Clear bit: Bit in andMask is zero, bit in xorMask is zero
//      Set bit: Bit in andMask is zero, bit in xorMask is one
//      No change: Bit in andMask is one, bit in xorMask is zero
//      Toggle bit: Bit in andMask is one, bit in xorMask is one
bool r4aSx1509RegisterModify(R4A_SX1509 * sx1509,
                             uint8_t registerAddress,
                             uint8_t andMask,
                             uint8_t xorMask,
                             Print *display)
{
    uint8_t data;

    // Read the register
    if (r4aSx1509RegisterRead(sx1509, registerAddress, &data, display))
    {
        // Toggle the bit or bits
        data = (data & andMask) ^ xorMask;

        // Write the new value to the register
        if (r4aSx1509RegisterWrite(sx1509, registerAddress, data, display))
            return true;
        else if (display)
            display->printf("ERROR: Failed to write 0x%2x to SX1509 register %d\r\n",
                            data, registerAddress);
    }
    else if (display)
        display->printf("ERROR: Failed to read SX1509 register %d\r\n", registerAddress);
    return false;
}

//*********************************************************************
// Read a SX1509 register
bool r4aSx1509RegisterRead(R4A_SX1509 * sx1509,
                           uint8_t registerAddress,
                           uint8_t * data,
                           Print * display)
{
    size_t bytesRead;

    // Read the register
    if (r4aI2cBusWriteRead(sx1509->_i2cBus,
                           sx1509->_i2cAddress,
                           &registerAddress,
                           sizeof(registerAddress),
                           data,
                           sizeof(*data),
                           &bytesRead) && (bytesRead == sizeof(*data)))
        return true;

    // Display the read error
    if (display)
        display->printf("ERROR: Failed to read from I2C address: 0x%03x\r\n",
                        sx1509->_i2cAddress);
    return false;
}

//*********************************************************************
// Write a SX1509 register
bool r4aSx1509RegisterWrite(R4A_SX1509 * sx1509,
                            uint8_t registerAddress,
                            uint8_t data,
                            Print * display)
{
    uint8_t buffer[2];

    // Write the register
    buffer[0] = registerAddress;
    buffer[1] = data;
    if (r4aI2cBusWrite(sx1509->_i2cBus,
                       sx1509->_i2cAddress,
                       buffer,
                       sizeof(buffer)))
        return true;

    // Display the write error
    if (display)
        display->printf("ERROR: Failed to write to I2C address: 0x%03x\r\n",
                        sx1509->_i2cAddress);
    return false;
}

//*********************************************************************
// Display TOn - TFall registers
void r4aSx1509DisplayTOnTfall(R4A_SX1509 * sx1509,
                              uint ioPort,
                              Print * display)
{
    uint8_t registerAddress;
    char registerName[32];

    // Validate the I/O port number
    if (ioPort > 15)
        return;

    // Get the register address
    registerAddress = SX1509_TON_ADDR[ioPort];

    // Display the registers
    sprintf(registerName, "RegTOn%d", ioPort);
    r4aSx1509RegisterDisplay(sx1509, registerAddress, registerName, display);

    registerAddress += 1;
    sprintf(registerName, "RegIon%d", ioPort);
    r4aSx1509RegisterDisplay(sx1509, registerAddress, registerName, display);

    registerAddress += 1;
    sprintf(registerName, "RegOff%d", ioPort);
    r4aSx1509RegisterDisplay(sx1509, registerAddress, registerName, display);

    if (((1 << ioPort) & SX1509_HAS_TRISE_TFALL) == 0)
        return;

    registerAddress += 1;
    sprintf(registerName, "RegTRise%d", ioPort);
    r4aSx1509RegisterDisplay(sx1509, registerAddress, registerName, display);

    registerAddress += 1;
    sprintf(registerName, "RegTFall%d", ioPort);
    r4aSx1509RegisterDisplay(sx1509, registerAddress, registerName, display);
}

//*********************************************************************
// Display the SX1509 registers
void r4aSx1509DisplayRegisters(R4A_SX1509 * sx1509,
                               Print * display)
{
    r4aSx1509RegisterDisplay(sx1509,    0, "RegInputDisableB", display);
    r4aSx1509RegisterDisplay(sx1509,    1, "RegInputDisableA", display);

    r4aSx1509RegisterDisplay(sx1509,    2, "RegLongSkewB", display);
    r4aSx1509RegisterDisplay(sx1509,    3, "RegLongSkewA", display);

    r4aSx1509RegisterDisplay(sx1509,    4, "RegLowDriveB", display);
    r4aSx1509RegisterDisplay(sx1509,    5, "RegLowDriveA", display);

    r4aSx1509RegisterDisplay(sx1509,    6, "RegPullUpB", display);
    r4aSx1509RegisterDisplay(sx1509,    7, "RegPullUpA", display);

    r4aSx1509RegisterDisplay(sx1509,    8, "RegPullDownB", display);
    r4aSx1509RegisterDisplay(sx1509,    9, "RegPullDownA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x0a, "RegOpenDrainB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x0b, "RegOpenDrainA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x0c, "RegPolarityB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x0d, "RegPolarityA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x0e, "RegDirB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x0f, "RegDirA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x10, "RegDataB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x11, "RegDataA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x12, "RegInterruptMaskB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x13, "RegInterruptMaskA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x14, "RegSenseHighB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x15, "RegSenseLowB", display);

    r4aSx1509RegisterDisplay(sx1509, 0x16, "RegSenseHighA", display);
    r4aSx1509RegisterDisplay(sx1509, 0x17, "RegSenseLowA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x18, "RegInterruptSourceB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x19, "RegInterruptSourceA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x1a, "RegEventStatusB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x1b, "RegEventStatusA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x1c, "RegLevelShifter1", display);
    r4aSx1509RegisterDisplay(sx1509, 0x1d, "RegLevelShifter2", display);

    r4aSx1509RegisterDisplay(sx1509, 0x1e, "RegClock", display);
    r4aSx1509RegisterDisplay(sx1509, 0x1f, "RegMisc", display);

    r4aSx1509RegisterDisplay(sx1509, 0x20, "RegLEDDriverEnableB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x21, "RegLEDDriverEnableA", display);

    r4aSx1509RegisterDisplay(sx1509, 0x22, "RegDebounceConfig", display);
    r4aSx1509RegisterDisplay(sx1509, 0x23, "RegDebounceEnableB", display);

    r4aSx1509RegisterDisplay(sx1509, 0x24, "RegDebounceEnableA", display);
    r4aSx1509RegisterDisplay(sx1509, 0x25, "RegKeyConfig1", display);

    r4aSx1509RegisterDisplay(sx1509, 0x26, "RegKeyConfig2", display);
    r4aSx1509RegisterDisplay(sx1509, 0x27, "RegKeyData1", display);

    r4aSx1509RegisterDisplay(sx1509, 0x28, "RegKeyData2", display);

    // Display the RegTOnxx, RegIOnxx, RegOffxx, RegTRisexx and RegTFallxx registers
    for (uint ioPort = 0; ioPort < 16; ioPort++)
        r4aSx1509DisplayTOnTfall(sx1509, ioPort, display);

    r4aSx1509RegisterDisplay(sx1509, 0x69, "RegHighInputB", display);
    r4aSx1509RegisterDisplay(sx1509, 0x6a, "RegHighInputA", display);
}
