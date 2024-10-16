/**********************************************************************
  I2C.cpp

  Robots-For-All (R4A)
  Generic I2C support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Globals
//****************************************

R4A_I2C_BUS * r4aI2cBus;

//*********************************************************************
// Enumerate the I2C bus
void R4A_I2C_BUS::enumerate(Print * display)
{
    bool deviceFound;
    int index;
    uint8_t mask;
    bool present;
    uint32_t timer;

    // Walk all of the I2C addresses
    deviceFound = false;
    for (uint8_t addr = 0; addr <= 0x7f; addr++)
    {
        present = false;
        timer = millis();
        if (enumerateDevice(addr))
        {
            present = true;
            if (deviceFound == false)
            {
                if (display)
                {
                    display->println();
                    display->println("I2C Devices:");
                }
                deviceFound = true;
            }

            // Look up the display name
            for (index = 0; index < _deviceTableEntries; index++)
                if (_deviceTable && (_deviceTable[index].deviceAddress == addr))
                {
                    deviceFound = true;
                    break;
                }

            if (display)
            {
                if (index < _deviceTableEntries)
                    display->printf("    0x%02x: %s\r\n", addr, _deviceTable[index].displayName);
                else if (addr == 0)
                    display->printf("    0x%02x: General Call\r\n", addr);
                else
                    display->printf("    0x%02x: ???\r\n", addr);
            }
        }
        else if ((millis() - timer) > 50)
        {
            if (display)
                display->println("ERROR: I2C bus not responding!");
            return;
        }

        // Update the present bit
        mask = 1 << (addr & 7);
        if (present)
            _present[addr / 8] |= mask;
        else
            _present[addr / 8] &= ~mask;
    }

    // Successful enumeration
    _enumerated = true;

    // Determine if any devices are on the bus
    if ((!deviceFound) && display)
        display->println("ERROR: No devices found on the I2C bus!");
}

//*********************************************************************
// Ping an I2C device and see if it responds
// Return true if device detected, false otherwise
bool R4A_I2C_BUS::enumerateDevice(uint8_t deviceAddress)
{
    int status;

    // Single thread the I2C requests
    r4aLockAcquire(&_lock);

    // Check for an I2C device
    _i2cBus->beginTransmission(deviceAddress);
    status = _i2cBus->endTransmission();

    // Release the lock
    r4aLockRelease(&_lock);

    // Return the I2C device found status
    if (status == 0)
        return true;
    return false;
}

//*********************************************************************
// Check if an I2C device was seen during the enumeration
// Return true if device detected, false otherwise
bool R4A_I2C_BUS::isDevicePresent(uint8_t deviceAddress)
{
    if (!_enumerated)
        enumerate(nullptr);
    return _present[deviceAddress / 8] & (1 << (deviceAddress & 7));
}

//*********************************************************************
// Send data to an I2C peripheral
// Return true upon success, false otherwise
bool R4A_I2C_BUS::write(uint8_t deviceI2cAddress,
                        const uint8_t * cmdBuffer,
                        size_t cmdByteCount,
                        const uint8_t * dataBuffer,
                        size_t dataByteCount,
                        Print * display,
                        bool releaseI2cBus)
{
    bool status;

    // Single thread the I2C requests
    r4aLockAcquire(&_lock);

    // Perform the I2C write operation
    status = writeWithLock(deviceI2cAddress,
                           cmdBuffer,
                           cmdByteCount,
                           dataBuffer,
                           dataByteCount,
                           display,
                           releaseI2cBus);

    // Release the lock
    r4aLockRelease(&_lock);

    // Return the write status
    return status;
}

//****************************************
// I2C menu API
//****************************************

//*********************************************************************
// Display the help text with mm and ssss
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aI2cMenuHelpAaRr(const struct _R4A_MENU_ENTRY * menuEntry,
                        const char * align,
                        Print * display)
{
    display->printf("%s aa rr: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Display the help text with mm and ssss
[[deprecated("Use r4aMenuHelpSuffix instead.")]]
void r4aI2cMenuHelpAaRrDd(const struct _R4A_MENU_ENTRY * menuEntry,
                          const char * align,
                          Print * display)
{
    display->printf("%s aa rr dd: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Get the I2C address and register numbers
bool r4aI2cMenuGetAddressRegister(const R4A_MENU_ENTRY * menuEntry,
                                  const char * command,
                                  int * values,
                                  uint8_t * i2cAddress,
                                  uint8_t * i2cRegister)
{
    int a;
    int r;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the values
    *values = sscanf(line.c_str(), "%02x %02x", &a, &r);

    // Determine if the values are within range
    if ((*values == 2)
        && (a >= 0)
        && (a <= 0x7f)
        && (r >= 0)
        && (r <= 0xff))
    {
        *i2cAddress = a;
        *i2cRegister = r;
        return true;
    }
    else if (*values == 1)
    {
        *i2cAddress = a;
        return true;
    }
    return false;
}

//*********************************************************************
// Get the device, register and data values
bool r4aI2cMenuGetAddressRegisterData(const R4A_MENU_ENTRY * menuEntry,
                                      const char * command,
                                      int * values,
                                      uint8_t * i2cAddress,
                                      uint8_t * i2cRegister,
                                      uint8_t * data)
{
    int a;
    int r;
    int w;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the values
    *values = sscanf(line.c_str(), "%2x %2x %2x", &a, &r, &w);

    // Determine if the values are within range
    if ((*values == 3)
        && (a >= 0)
        && (a <= 0x7f)
        && (r >= 0)
        && (r <= 0xff)
        && (w >= 0)
        && (w <= 0xff))
    {
        *i2cAddress = a;
        *i2cRegister = r;
        *data = w;
        return true;
    }
    else if (*values == 2)
    {
        *i2cAddress = a;
        *data = r;
        return true;
    }
    else if (*values == 1)
        *i2cAddress = a;
    return false;
}

//*********************************************************************
// Enumerate the I2C bus
void r4aI2cMenuEnumerate(const R4A_MENU_ENTRY * menuEntry,
                         const char * command,
                         Print * display)
{
    r4aI2cBus->enumerate(display);
}

//*********************************************************************
// Read data from the I2C device
void r4aI2cMenuRead(const R4A_MENU_ENTRY * menuEntry,
                    const char * command,
                    Print * display)
{
    int bytesRead;
    uint8_t data;
    uint8_t i2cAddress;
    uint8_t i2cRegister;
    int values;

    // Parse the command line
    if (r4aI2cMenuGetAddressRegister(menuEntry, command, &values, &i2cAddress, &i2cRegister))
    {
        bytesRead = r4aI2cBus->read(i2cAddress,
                                    (values == 2) ? &i2cRegister : nullptr,
                                    (values == 2) ? sizeof(i2cRegister) : 0,
                                    &data,
                                    sizeof(data),
                                    nullptr,
                                    true);       // End of transaction
        if (bytesRead != sizeof(data))
            display->println("Failed to read register!");
        else if (values == 1)
            display->printf("0x%02x: 0x%02x (%d)\r\n",
                            i2cAddress,
                            data, data);
        else
            display->printf("0x%02x[0x%02x]: 0x%02x (%d)\r\n",
                            i2cAddress,
                            i2cRegister,
                            data, data);
    }
    else if (values <= 0)
        display->println("Please specify the I2C address (0 - 0x7f) for aa");
}

//*********************************************************************
// Write data to the I2C device
void r4aI2cMenuWrite(const R4A_MENU_ENTRY * menuEntry,
                     const char * command,
                     Print * display)
{
    int bytesWritten;
    uint8_t data;
    uint8_t i2cAddress;
    uint8_t i2cRegister;
    int values;

    // Parse the command line
    if (r4aI2cMenuGetAddressRegisterData(menuEntry, command, &values, &i2cAddress, &i2cRegister, &data))
    {
        bytesWritten = r4aI2cBus->write(i2cAddress,
                                        (values == 3) ? &i2cRegister : nullptr,
                                        (values == 3) ? sizeof(i2cRegister) : 0,
                                        &data,
                                        sizeof(data),
                                        nullptr,
                                        true);       // End of transaction
        if (bytesWritten != (values - 1))
            display->println("Failed to write register!");
    }
    else if (values <= 0)
        display->println("Please specify the I2C address (0 - 0x7f) for aa");
    else if (values == 1)
        display->println("Please specify the I2C register (0 - 0xff) for rr");
}
