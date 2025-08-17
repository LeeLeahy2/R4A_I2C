/**********************************************************************
  Camera.cpp

  Robots-For-All (R4A)
  Common camera support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Constants
//****************************************

const R4A_CAMERA_FRAME r4aCameraFrameFormats[] =
{
    // Width Height   Name      Symbol
    {  40,       30, "40x30",   R4A_FRAME_SIZE_40x30},
    {  80,       60, "80x60",   R4A_FRAME_SIZE_80x60},
    {  96,       96, "96x96",   R4A_FRAME_SIZE_96x96},
    { 100,       75, "100x75",  R4A_FRAME_SIZE_100x75},
    { 128,      128, "128x128", R4A_FRAME_SIZE_128x128},
    { 160,      120, "QQVGA",   R4A_FRAME_SIZE_QQVGA},
    { 176,      144, "QCIF",    R4A_FRAME_SIZE_QCIF},
    { 200,      150, "200x150", R4A_FRAME_SIZE_200x150},
    { 240,      176, "HQVGA",   R4A_FRAME_SIZE_HQVGA},
    { 240,      240, "240x240", R4A_FRAME_SIZE_240x240},
    { 320,      240, "QVGA",    R4A_FRAME_SIZE_QVGA},
    { 320,      320, "320x320", R4A_FRAME_SIZE_320x320},
    { 400,      296, "CIF",     R4A_FRAME_SIZE_CIF},
    { 400,      300, "400x300", R4A_FRAME_SIZE_400x300},
    { 480,      320, "HVGA",    R4A_FRAME_SIZE_HVGA},
    { 640,      480, "VGA",     R4A_FRAME_SIZE_VGA},
    { 720,     1280, "P_HD",    R4A_FRAME_SIZE_P_HD},
    { 800,      600, "SVGA",    R4A_FRAME_SIZE_SVGA},
    { 864,     1536, "P_3MP",   R4A_FRAME_SIZE_P_3MP},
    {1024,      768, "XGA",     R4A_FRAME_SIZE_XGA},
    {1080,     1920, "P_FHD",   R4A_FRAME_SIZE_P_FHD},
    {1280,      720, "HD",      R4A_FRAME_SIZE_HD},
    {1280,     1024, "SXGA",    R4A_FRAME_SIZE_SXGA},
    {1600,     1200, "UXGA",    R4A_FRAME_SIZE_UXGA},
    {1920,     1080, "FHD",     R4A_FRAME_SIZE_FHD},
    {2048,     1536, "QXGA",    R4A_FRAME_SIZE_QXGA},
    {2560,     1440, "QHD",     R4A_FRAME_SIZE_QHD},
    {2560,     1600, "WQXGA",   R4A_FRAME_SIZE_WQXGA},
    {2560,     1920, "QSXGA",   R4A_FRAME_SIZE_QSXGA},
    {2592,     1944, "5MP",     R4A_FRAME_SIZE_5MP},
};
const int r4aCameraFrameFormatsEntries = sizeof(r4aCameraFrameFormats)
                                       / sizeof(r4aCameraFrameFormats[0]);

// Define the supported camera formats
const R4A_CAMERA_PIXEL r4aCameraPixelFormats[] =
{
    //  Name      Color bits  Symbol
    {"GRAYSCALE", false,   8, R4A_PIXEL_FORMAT_GRAYSCALE},
    {"JPEG",      true,   24, R4A_PIXEL_FORMAT_JPEG},
    {"RAW",       true,   32, R4A_PIXEL_FORMAT_RAW},
    {"RGB444",    true,   12, R4A_PIXEL_FORMAT_RGB444},
    {"RGB555",    true,   16, R4A_PIXEL_FORMAT_RGB555},
    {"RGB565",    true,   16, R4A_PIXEL_FORMAT_RGB565},
    {"RGB888",    true,   24, R4A_PIXEL_FORMAT_RGB888},
    {"YUV420",    true,    6, R4A_PIXEL_FORMAT_YUV420},
    {"YUV422",    true,    8, R4A_PIXEL_FORMAT_YUV422},
};
const int r4aCameraPixelFormatsEntries = sizeof(r4aCameraPixelFormats)
                                       / sizeof(r4aCameraPixelFormats[0]);

//*********************************************************************
// Lookup the frame size
const R4A_CAMERA_FRAME * r4aCameraFindFrameSize(R4A_FRAME_SIZE_t frameSize)
{
    // Walk the list of frame formats
    for (int index = 0; index < r4aCameraFrameFormatsEntries; index++)
        if (frameSize == r4aCameraFrameFormats[index].frameSize)
            return &r4aCameraFrameFormats[index];
    return nullptr;
}

//*********************************************************************
// Lookup the pixel format
const R4A_CAMERA_PIXEL * r4aCameraFindPixelFormat(R4A_PIXEL_FORMAT_t pixelFormat)
{
    // Walk the list of frame formats
    for (int index = 0; index < r4aCameraPixelFormatsEntries; index++)
        if (pixelFormat == r4aCameraPixelFormats[index].format)
            return &r4aCameraPixelFormats[index];
    return nullptr;
}

//*********************************************************************
// Verify the enum values against the corresponding tables
void r4aCameraVerifyTables()
{
    int index;

    // Frame sizes
    if (r4aCameraFrameFormatsEntries != R4A_FRAME_SIZE_MAX)
        r4aReportFatalError("Fix enum R4A_FRAME_SIZE_t and r4aCameraFrameFormats!");
    for (index = 0; index < r4aCameraFrameFormatsEntries; index++)
        if (r4aCameraFrameFormats[index].frameSize != index)
            r4aReportFatalError("Fix enum R4A_FRAME_SIZE_t and r4aCameraFrameFormats order!");

    // Pixel formats
    if (r4aCameraPixelFormatsEntries != R4A_PIXEL_FORMAT_MAX)
        r4aReportFatalError("Fix enum R4A_PIXEL_FORMAT_t and r4aCameraPixelFormats!");
    for (index = 0; index < r4aCameraPixelFormatsEntries; index++)
        if (r4aCameraPixelFormats[index].format != index)
            r4aReportFatalError("Fix enum R4A_PIXEL_FORMAT_t and r4aCameraPixelFormats order!");
}
