/**********************************************************************
  Camera.cpp

  Robots-For-All (R4A)
  Common camera support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Constants
//****************************************

const R4A_CAMERA_FRAME r4aCameraFrameFormat[] =
{
    // Entries sorted by width, height
    // Width Height  Ratio    Name      Symbol
    {  40,       30,  4,  3,  "40x30",   R4A_FRAME_SIZE_40x30},
    {  80,       60,  4,  3,  "80x60",   R4A_FRAME_SIZE_80x60},
    {  96,       96,  1,  1,  "96x96",   R4A_FRAME_SIZE_96x96},
    { 100,       75,  4,  3,  "100x75",  R4A_FRAME_SIZE_100x75},
    { 128,      128,  1,  1,  "128x128", R4A_FRAME_SIZE_128x128},
    { 160,      120,  4,  3,  "QQVGA",   R4A_FRAME_SIZE_QQVGA},
    { 176,      144,  5,  4,  "QCIF",    R4A_FRAME_SIZE_QCIF},
    { 200,      150,  4,  3,  "200x150", R4A_FRAME_SIZE_200x150},
    { 240,      176,  4,  3,  "HQVGA",   R4A_FRAME_SIZE_HQVGA},
    { 240,      240,  1,  1,  "240x240", R4A_FRAME_SIZE_240x240},
    { 320,      240,  4,  3,  "QVGA",    R4A_FRAME_SIZE_QVGA},
    { 320,      320,  1,  1,  "320x320", R4A_FRAME_SIZE_320x320},
    { 400,      296,  4,  3,  "CIF",     R4A_FRAME_SIZE_CIF},
    { 400,      300,  4,  3,  "400x300", R4A_FRAME_SIZE_400x300},
    { 480,      320,  3,  2,  "HVGA",    R4A_FRAME_SIZE_HVGA},
    { 640,      480,  4,  3,  "VGA",     R4A_FRAME_SIZE_VGA},
    { 720,     1280,  9, 16,  "P_HD",    R4A_FRAME_SIZE_P_HD},
    { 800,      600,  4,  3,  "SVGA",    R4A_FRAME_SIZE_SVGA},
    { 864,     1536,  9, 16,  "P_3MP",   R4A_FRAME_SIZE_P_3MP},
    {1024,      768,  4,  3,  "XGA",     R4A_FRAME_SIZE_XGA},
    {1080,     1920,  9, 16,  "P_FHD",   R4A_FRAME_SIZE_P_FHD},
    {1280,      720, 16,  9,  "HD",      R4A_FRAME_SIZE_HD},
    {1280,     1024,  5,  4,  "SXGA",    R4A_FRAME_SIZE_SXGA},
    {1600,     1200,  4,  3,  "UXGA",    R4A_FRAME_SIZE_UXGA},
    {1920,     1080, 16,  9,  "FHD",     R4A_FRAME_SIZE_FHD},
    {2048,     1536,  4,  3,  "QXGA",    R4A_FRAME_SIZE_QXGA},
    {2560,     1440, 16,  9,  "QHD",     R4A_FRAME_SIZE_QHD},
    {2560,     1600, 16, 10,  "WQXGA",   R4A_FRAME_SIZE_WQXGA},
    {2560,     1920,  4,  3,  "QSXGA",   R4A_FRAME_SIZE_QSXGA},
    {2592,     1944,  4,  3,  "5MP",     R4A_FRAME_SIZE_5MP},
};
const int r4aCameraFrameFormatEntries = sizeof(r4aCameraFrameFormat)
                                      / sizeof(r4aCameraFrameFormat[0]);

// Define the supported camera formats
const R4A_CAMERA_PIXEL r4aCameraPixelFormat[] =
{
    // Entries sorted by bits, resolution, color
    // Name      Color  Bits  Symbol
    {"RAW",       true,   32, R4A_PIXEL_FORMAT_RAW},
    {"RGB888",    true,   24, R4A_PIXEL_FORMAT_RGB888},
    {"JPEG",      true,   24, R4A_PIXEL_FORMAT_JPEG},
    {"RGB565",    true,   16, R4A_PIXEL_FORMAT_RGB565},
    {"RGB555",    true,   16, R4A_PIXEL_FORMAT_RGB555},
    {"RGB444",    true,   12, R4A_PIXEL_FORMAT_RGB444},
    {"YUV422",    true,    8, R4A_PIXEL_FORMAT_YUV422},
    {"GRAYSCALE", false,   8, R4A_PIXEL_FORMAT_GRAYSCALE},
    {"YUV420",    true,    6, R4A_PIXEL_FORMAT_YUV420},
};
const int r4aCameraPixelFormatEntries = sizeof(r4aCameraPixelFormat)
                                      / sizeof(r4aCameraPixelFormat[0]);

//****************************************
// Locals
//****************************************

volatile int32_t r4aCameraUsers;

//*********************************************************************
// Lookup the frame size
const R4A_CAMERA_FRAME * r4aCameraFindFrameSize(R4A_FRAME_SIZE_t r4aFrameSize)
{
    // Walk the list of frame formats
    for (int index = 0; index < r4aCameraFrameFormatEntries; index++)
        if (r4aFrameSize == r4aCameraFrameFormat[index]._r4aFrameSize)
            return &r4aCameraFrameFormat[index];
    return nullptr;
}

//*********************************************************************
// Lookup the pixel format
const R4A_CAMERA_PIXEL * r4aCameraFindPixelFormat(R4A_PIXEL_FORMAT_t pixelFormat)
{
    // Walk the list of frame formats
    for (int index = 0; index < r4aCameraPixelFormatEntries; index++)
        if (pixelFormat == r4aCameraPixelFormat[index]._format)
            return &r4aCameraPixelFormat[index];
    return nullptr;
}

//*********************************************************************
// Verify the enum values against the corresponding tables
void r4aCameraVerifyTables()
{
    uint64_t found;
    int index;

    // Check for duplicates in the r4aCameraFrameFormat table
    found = 0;
    for (index = 0; index < r4aCameraFrameFormatEntries; index++)
    {
        if (found & (1 << r4aCameraFrameFormat[index]._r4aFrameSize))
        {
            Serial.printf("ERROR: Duplicate _frameSize entry at r4aCameraFrameFormat[%d]\r\n", index);
            r4aReportFatalError("Duplicate _frameSize entry in r4aCameraFrameFormat table!");
        }
        else
            found |= 1 << r4aCameraFrameFormat[index]._r4aFrameSize;
    }

    // Verify the r4aCameraFrameFormat table size
    if (r4aCameraFrameFormatEntries != R4A_FRAME_SIZE_MAX)
    {
        Serial.printf("ERROR: Too %s entries in r4aCameraFrameFormat table!\r\n",
                      r4aCameraFrameFormatEntries > R4A_FRAME_SIZE_MAX ? "many" : "few");
        r4aReportFatalError("Fix enum R4A_FRAME_SIZE_t and r4aCameraFrameFormat!");
    }

    // Check for duplicates in the r4aCameraPixelFormat table
    found = 0;
    for (index = 0; index < r4aCameraPixelFormatEntries; index++)
    {
        if (found & (1 << r4aCameraPixelFormat[index]._format))
        {
            Serial.printf("ERROR: Duplicate _format entry at r4aCameraPixelFormat[%d]\r\n", index);
            r4aReportFatalError("Duplicate _format entry in r4aCameraPixelFormat table!");
        }
        else
            found |= 1 << r4aCameraPixelFormat[index]._format;
    }

    // Verify the r4aCameraPixelFormat table size
    if (r4aCameraPixelFormatEntries != R4A_PIXEL_FORMAT_MAX)
    {
        Serial.printf("ERROR: Too %s entries in r4aCameraPixelFormat table!\r\n",
                      r4aCameraPixelFormatEntries > R4A_PIXEL_FORMAT_MAX ? "many" : "few");
        r4aReportFatalError("Fix enum R4A_PIXEL_FORMAT_t and r4aCameraPixelFormat!");
    }
}
