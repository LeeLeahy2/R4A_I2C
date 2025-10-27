/**********************************************************************
  I2C.cpp

  Robots-For-All (R4A)
  Generic I2C support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Enumerate the I2C bus
void r4aI2cBusEnumerate(R4A_I2C_BUS * i2cBus, Print * display)
{
    bool deviceFound;
    int index;
    uint8_t mask;
    bool present;
    uint32_t timer;

    // Walk all of the I2C addresses
    deviceFound = false;
    for (R4A_I2C_ADDRESS_t addr = 0; addr < R4A_I2C_ADDRESSES_7_BIT; addr++)
    {
        present = false;
        timer = millis();
        if (r4aI2cBusEnumerateDevice(i2cBus, addr))
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
            for (index = 0; index < i2cBus->_deviceTableEntries; index++)
                if (i2cBus->_deviceTable && (i2cBus->_deviceTable[index].i2cAddress == addr))
                {
                    deviceFound = true;
                    break;
                }

            if (display)
            {
                if (index < i2cBus->_deviceTableEntries)
                    display->printf("    0x%03x: %s\r\n", addr, i2cBus->_deviceTable[index].displayName);
                else
                    display->printf("    0x%03x: ???\r\n", addr);
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
            i2cBus->_present[addr / 8] |= mask;
        else
            i2cBus->_present[addr / 8] &= ~mask;
    }

    // Successful enumeration
    i2cBus->_enumerated = true;

    // Determine if any devices are on the bus
    if ((!deviceFound) && display)
        display->println("ERROR: No devices found on the I2C bus!");
}

//*********************************************************************
// Enumerate the 10-bit I2C bus
void r4aI2cBusEnumerate10Bit(R4A_I2C_BUS * i2cBus, Print * display)
{
    bool deviceFound;
    int index;
    uint8_t mask;
    bool present;
    uint32_t timer;

    // Walk all of the I2C addresses
    deviceFound = false;
    for (R4A_I2C_ADDRESS_t addr = 0; addr < R4A_I2C_ADDRESSES_10_BIT; addr++)
    {
        present = false;
        timer = millis();
        if (r4aI2cBusEnumerateDevice(i2cBus, addr))
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
            for (index = 0; index < i2cBus->_deviceTableEntries; index++)
                if (i2cBus->_deviceTable && (i2cBus->_deviceTable[index].i2cAddress == addr))
                {
                    deviceFound = true;
                    break;
                }

            if (display)
            {
                if (index < i2cBus->_deviceTableEntries)
                    display->printf("    0x%03x: %s\r\n", addr, i2cBus->_deviceTable[index].displayName);
                else if (addr == 0)
                    display->printf("    0x%03x: General Call\r\n", addr);
                else
                    display->printf("    0x%03x: ???\r\n", addr);
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
            i2cBus->_present[addr / 8] |= mask;
        else
            i2cBus->_present[addr / 8] &= ~mask;
    }

    // Successful enumeration
    i2cBus->_enumerated = true;

    // Determine if any devices are on the bus
    if ((!deviceFound) && display)
        display->println("ERROR: No devices found on the I2C bus!");
}

//*********************************************************************
// Check if an I2C device was seen during the enumeration
// Return true if device detected, false otherwise
bool r4aI2cBusIsDevicePresent(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress)
{
    if ((i2cBus == nullptr) || (i2cAddress >= R4A_I2C_ADDRESSES))
        return false;
    if (!i2cBus->_enumerated)
        r4aI2cBusEnumerate(i2cBus, nullptr);
    return i2cBus->_present[i2cAddress / 8] & (1 << (i2cAddress & 7));
}

//*********************************************************************
// Issue a software reset to the I2C devices
bool r4aI2cCallSwReset(R4A_I2C_BUS * i2cBus, Print * display, Print * debug)
{
    uint8_t data;
    bool success;

    // Reset the devices on the I2C bus
    data = R4A_I2C_SWRST;
    success = r4aI2cBusWrite(i2cBus,
                             R4A_I2C_GENERAL_CALL_DEVICE_ADDRESS,
                             &data,
                             sizeof(data),
                             debug);
    if (success == false)
    {
        const char * errorMessage = "ERROR: Failed to reset the I2C bus!\r\n";
        if (display)
            display->printf(errorMessage);
        else if (debug)
            debug->printf(errorMessage);
    }
    return success;
}

//****************************************
// I2C menu API
//****************************************

//*********************************************************************
// Get the I2C address and register numbers
bool r4aI2cMenuGetAddressRegister(const R4A_MENU_ENTRY * menuEntry,
                                  const char * command,
                                  int * values,
                                  R4A_I2C_ADDRESS_t * i2cAddress,
                                  uint8_t * i2cRegister)
{
    int a;
    int r;

    // Get the parameter name
    String line = r4aMenuGetParameters(menuEntry, command);

    // Get the values
    *values = sscanf(line.c_str(), "%x %x", &a, &r);

    // Determine if the values are within range
    if ((*values == 2)
        && (a >= 0)
        && (a < R4A_I2C_ADDRESSES)
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
                                      R4A_I2C_ADDRESS_t * i2cAddress,
                                      uint8_t * i2cRegister,
                                      uint8_t * data)
{
    int a;
    int r;
    int w;

    // Get the parameter name
    String line = r4aMenuGetParameters(menuEntry, command);

    // Get the values
    *values = sscanf(line.c_str(), "%x %x %x", &a, &r, &w);

    // Determine if the values are within range
    if ((*values == 3)
        && (a >= 0)
        && (a < R4A_I2C_ADDRESSES)
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
    r4aI2cBusEnumerate(r4aI2cBus, display);
}

//*********************************************************************
// Read data from the I2C device
void r4aI2cMenuRead(const R4A_MENU_ENTRY * menuEntry,
                    const char * command,
                    Print * display)
{
    uint8_t data;
    R4A_I2C_ADDRESS_t i2cAddress;
    uint8_t i2cRegister;
    int values;

    do
    {
        // Parse the command line
        if (r4aI2cMenuGetAddressRegister(menuEntry,
                                         command,
                                         &values,
                                         &i2cAddress,
                                         &i2cRegister))
        {
            // Set the register address
            if (values == 2)
            {
                if (r4aI2cBusWriteRead(r4aI2cBus,
                                       i2cAddress,
                                       &i2cRegister,
                                       sizeof(i2cRegister),
                                       &data,
                                       sizeof(data),
                                       nullptr,
                                       display) == false)
                {
                    display->println("Failed to read register!");
                    break;
                }
                display->printf("0x%03x[0x%02x]: 0x%02x (%d)\r\n",
                                i2cAddress,
                                i2cRegister,
                                data, data);
            }
            else
            {
                // Read the data byte
                if (r4aI2cBusRead(r4aI2cBus,
                                  i2cAddress,
                                  &data,
                                  sizeof(data),
                                  nullptr,
                                  display) == false)
                {
                    display->println("Failed to read register!");
                    break;
                }
                display->printf("0x%03x: 0x%02x (%d)\r\n",
                                i2cAddress,
                                data, data);
            }
        }
        else if (values <= 0)
            display->printf("Please specify the I2C address (0 - 0x%03x) for aa",
                            R4A_I2C_ADDRESSES - 1);
    } while (0);
}

//*********************************************************************
// Write data to the I2C device
void r4aI2cMenuWrite(const R4A_MENU_ENTRY * menuEntry,
                     const char * command,
                     Print * display)
{
    uint8_t data[1 + 16]; // Optional register address followed by data bytes
    size_t dataBytes;
    R4A_I2C_ADDRESS_t i2cAddress;
    String line;
    int values;

    do
    {
        // Get the parameter name
        String line = r4aMenuGetParameters(menuEntry, command);
        line.trim();

        // Get the I2C address
        if (sscanf(line.c_str(), "%x", &i2cAddress) == 0)
        {
            if (display)
                display->printf("Please specify the I2C address (0 - 0x%03x) for aa",
                                R4A_I2C_ADDRESSES - 1);
            break;
        }

        dataBytes = 0;
        while (1)
        {
            // Remove the hex value
            while (((line.c_str()[0] >= '0') && (line.c_str()[0] <= '9'))
                || ((line.c_str()[0] >= 'a') && (line.c_str()[0] <= 'f'))
                || ((line.c_str()[0] >= 'A') && (line.c_str()[0] <= 'F')))
            {
                line = line.substring(1);
            }

            // Remove the white space
            line.trim();

            if ((line.c_str()[0] == 0) || (line.c_str()[0] == '\n') || (line.c_str()[0] == '\r'))
                break;

            // Get the next data byte
            if (sscanf(line.c_str(), "%x", &data[dataBytes]) == 0)
            {
                if (display)
                    display->printf("Invalid data byte [%d]\r\n", dataBytes);
                break;
            }

            // Account for this data byte
            dataBytes += 1;
            if (dataBytes == sizeof(data))
                break;
        }

        // Verify that there is at least one data byte
        if (dataBytes == 0)
        {
            if (display)
                display->println("Please specify a data byte or the I2C register (0 - 0xff) for rr");
            break;
        }

        // Send the data via I2C
        if (r4aI2cBusWrite(r4aI2cBus,
                           i2cAddress,
                           data,
                           dataBytes,
                           nullptr))

            // Successful write
            break;

        // Display the error
        display->printf("ERROR: Failed to write I2C data to 0x%03x!",
                        i2cAddress);
    } while (0);
}
