/**********************************************************************
  VK16K33.cpp

  Robots-For-All (R4A)
  LED controller support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Constants
//****************************************

#define R4A_VK16K33_CMD_DATA_ADDRESS        0x00
#define R4A_VK16K33_CMD_SYSTEM_SET          0x20
#define R4A_VK16K33_CMD_DISPLAY_SET         0x80
#define R4A_VK16K33_CMD_DISPLAY_BRIGHTNESS  0xe0

#define R4A_VK16K33_CSS_ON              0x01    // Turn on the controller

#define R4A_VK16K33_CDS_ON              0x01    // Turn on the display
#define R4A_VK16K33_CDS_BLINK_OFF       0       // Disable blinking
#define R4A_VK16K33_CDS_BLINK_2HZ       0x02    // 2 Hz blink frequency
#define R4A_VK16K33_CDS_BLINK_1HZ       0x02    // 1 Hz blink frequency
#define R4A_VK16K33_CDS_BLINK_0_5HZ     0x02    // 0.5 Hz blink frequency

#define R4A_VK16K33_CDB_1_16            0   // 1 / 16
#define R4A_VK16K33_CDB_2_16            1   // 2 / 16
#define R4A_VK16K33_CDB_3_16            2   // 3 / 16
#define R4A_VK16K33_CDB_4_16            3   // 4 / 16
#define R4A_VK16K33_CDB_5_16            4   // 5 / 16
#define R4A_VK16K33_CDB_6_16            5   // 6 / 16
#define R4A_VK16K33_CDB_7_16            6   // 7 / 16
#define R4A_VK16K33_CDB_8_16            7   // 8 / 16
#define R4A_VK16K33_CDB_9_16            8   // 9 / 16
#define R4A_VK16K33_CDB_10_16           9   // 10 / 16
#define R4A_VK16K33_CDB_11_16           10  // 11 / 16
#define R4A_VK16K33_CDB_12_16           11  // 12 / 16
#define R4A_VK16K33_CDB_13_16           12  // 13 / 16
#define R4A_VK16K33_CDB_14_16           13  // 14 / 16
#define R4A_VK16K33_CDB_15_16           14  // 15 / 16
#define R4A_VK16K33_CDB_16_16           15  // 16 / 16

//*********************************************************************
// Set the brightness (0-15)
bool r4aVk16k33Brightness(R4A_VK16K33 * vk16k33,
                          uint8_t brightness,
                          Print * display)
{
    uint8_t cmd;
    bool success = false;

    // Use the Display Brightness command to set the pulse width, see
    // VK16K33 specification v1.2, page 27
    cmd = R4A_VK16K33_CMD_DISPLAY_BRIGHTNESS | (brightness & 0xf);
    success = r4aI2cBusWrite(vk16k33->i2cBus,
                             vk16k33->i2cAddress,
                             &cmd,
                             sizeof(cmd),
                             nullptr,
                             0,
                             display);
    if ((!success) && display)
        display->printf("ERROR: Failed to set VK16K33 brightness!\r\n");
    return success;
}

//*********************************************************************
// Clear the RAM buffer
void r4aVk16k33BufferClear(R4A_VK16K33 * vk16k33)
{
    memset(&vk16k33->pixels[R4A_VK16K33_PIXEL_OFFSET], 0, R4A_VK16K33_MAX_COLUMNS);
}

//*********************************************************************
// Fill the RAM buffer
void r4aVk16k33BufferFill(R4A_VK16K33 * vk16k33, uint8_t data)
{
    memset(&vk16k33->pixels[R4A_VK16K33_PIXEL_OFFSET], data, R4A_VK16K33_MAX_COLUMNS);
}

//*********************************************************************
// Turn on the display
bool r4aVk16k33DisplayOn(R4A_VK16K33 * vk16k33, Print * display)
{
    uint8_t cmd;
    bool success = false;

    // Use the Display Set command to turn on the display and disable
    // blinking, see VK16K33 specification v1.2, page 27
    cmd = R4A_VK16K33_CMD_DISPLAY_SET
        | R4A_VK16K33_CDS_BLINK_OFF
        | R4A_VK16K33_CDS_ON;
    success = r4aI2cBusWrite(vk16k33->i2cBus,
                             vk16k33->i2cAddress,
                             &cmd,
                             sizeof(cmd),
                             nullptr,
                             0,
                             display);
    if (!success)
        Serial.printf("ERROR: Failed to turn on VK16K33 display!\r\n");
    return success;
}

//*********************************************************************
// Copy the RAM buffer to the display
// Start bit, I2C device address, ACK, register address, ACK, 16 data bytes
// with ACKs and a stop bit, all at 400 KHz
// ~410 uSec = (1+8+1+8+1+((8+1)×16)+1)÷(400×1000)
bool r4aVk16k33DisplayPixels(R4A_VK16K33 * vk16k33, Print * display)
{
    bool success = false;

    do
    {
        // Concatenate the command and pixel data
        // Copy the RAM buffer to the display
        // HT16K33 specification v1.10, page 30
        vk16k33->pixels[0] = R4A_VK16K33_CMD_DATA_ADDRESS | 0;
        success = r4aI2cBusWrite(vk16k33->i2cBus,
                                 vk16k33->i2cAddress,
                                 nullptr,
                                 0,
                                 vk16k33->pixels,
                                 sizeof(vk16k33->pixels),
                                 display);
        if (!success)
        {
            if (display)
                display->printf("ERROR: Failed to write VK16K33 pixel data!\r\n");
            break;
        }
    } while (0);
    return success;
}

//*********************************************************************
// Turn on the VK16K33 LED controller
bool r4aVk16k33On(R4A_VK16K33 * vk16k33, Print * display)
{
    uint8_t cmd;
    bool success;

    // Use the System Set command to turn on the controller, see
    // VK16K33 specification v1.2, page 27
    cmd = R4A_VK16K33_CMD_SYSTEM_SET | R4A_VK16K33_CSS_ON;
    success = r4aI2cBusWrite(vk16k33->i2cBus,
                             vk16k33->i2cAddress,
                             &cmd,
                             sizeof(cmd),
                             nullptr,
                             0,
                             display);
    if (!success)
        Serial.printf("ERROR: Failed to turn on VK16K33!\r\n");
    return success;
}

//*********************************************************************
// Clear a pixel in the RAM buffer
bool r4aVk16k33PixelClear(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t row)
{
    do
    {
        // Verify the column and row
        if (column >= vk16k33->columns)
        {
            Serial.printf("ERROR: Invalid column number, must be <= %d\r\n",
                          vk16k33->columns);
            break;
        }
        if (row >= vk16k33->rows)
        {
            Serial.printf("ERROR: Invalid row number, must be <= %d\r\n",
                          vk16k33->rows);
            break;
        }

        // Clear the pixel
        uint8_t bitMask = 1 << row;
        vk16k33->pixels[R4A_VK16K33_PIXEL_OFFSET + column] &= ~bitMask;
        return true;
    } while (0);
    return false;
}

//*********************************************************************
// Set a pixel in the RAM buffer
bool r4aVk16k33PixelSet(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t row)
{
    do
    {
        // Verify the column and row
        if (column >= vk16k33->columns)
        {
            Serial.printf("ERROR: Invalid column number, must be <= %d\r\n",
                          vk16k33->columns);
            break;
        }
        if (row >= vk16k33->rows)
        {
            Serial.printf("ERROR: Invalid row number, must be <= %d\r\n",
                          vk16k33->rows);
            break;
        }

        // Set the pixel
        uint8_t bitMask = 1 << row;
        vk16k33->pixels[R4A_VK16K33_PIXEL_OFFSET + column] |= bitMask;
        return true;
    } while (0);
    return false;
}

//*********************************************************************
// Initialize the VK16K33
bool r4aVk16k33Setup(R4A_VK16K33 * vk16k33, Print * display)
{
    uint8_t cmd;
    bool success = false;

    do
    {
        // Verify the number of columns and rows
        if (vk16k33->columns > R4A_VK16K33_MAX_COLUMNS)
        {
            if (display)
                display->printf("ERROR: Too many columns, columns <= %d\r\n",
                                R4A_VK16K33_MAX_COLUMNS);
            break;
        }
        if (vk16k33->rows > R4A_VK16K33_MAX_ROWS)
        {
            if (display)
                display->printf("ERROR: Too many rows, rows <= %d\r\n",
                                R4A_VK16K33_MAX_ROWS);
            break;
        }

        // Turn on the controller
        success = r4aVk16k33On(vk16k33, display);
        if (!success)
            break;

        // Clear the display buffer
        r4aVk16k33BufferClear(vk16k33);
        success = r4aVk16k33DisplayPixels(vk16k33, display);
        if (!success)
            break;

        // Turn on the display, start the scanning of the LEDs
        success = r4aVk16k33DisplayOn(vk16k33, display);
        if (!success)
            break;

        // Use the Display Brightness command to set the pulse width, see
        // VK16K33 specification v1.2, page 27
        success = r4aVk16k33Brightness(vk16k33, vk16k33->brightness, display);
        if (!success)
            break;
    } while (0);
    return success;
}

//*********************************************************************
// Write a column of eight pixels in the RAM buffer
bool r4aVk16k33WriteColumn(R4A_VK16K33 * vk16k33, uint8_t column, uint8_t data)
{
    // Verify the column and row
    if (column >= vk16k33->columns)
    {
        Serial.printf("ERROR: Invalid column number, must be <= %d\r\n",
                      vk16k33->columns);
        return false;
    }

    // Set the pixels
    vk16k33->pixels[R4A_VK16K33_PIXEL_OFFSET + column] = data;
    return true;
}
