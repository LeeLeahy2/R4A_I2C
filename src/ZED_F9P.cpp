/**********************************************************************
  ZED_F9P.cpp

  Support for the u-blox ZED-F9P GNSS receiver
**********************************************************************/

#include <R4A_I2C.h>

//****************************************
// Globals - Parameters
//****************************************

bool r4aZedF9pDisplayTime;          // Display time data
bool r4aZedF9pDisplaySiv;           // Display satellites-in-view
bool r4aZedF9pDisplayHpaLatLong;    // Display the latitude and longitude
bool r4aZedF9pDisplayAltitude;      // Display the altitude
bool r4aZedF9pDisplayFixType;       // Display the fix type
uint32_t r4aZedF9pLocationDisplayMsec = 1000; // 0 = Off, Interval to display the location
uint32_t r4aZedF9pPollMsec = 100;   // I2C polling interval for the GNSS receiver
bool r4aZedF9pUnitsFeetInches;      // Display in feet and inches .vs. meters

//*********************************************************************
// Destructor
R4A_ZED_F9P::~R4A_ZED_F9P()
{
    if (_altitudeArray)
    {
        free(_altitudeArray);
        _altitudeArray = nullptr;
    }
    if (_horizontalAccuracyArray)
    {
        free(_horizontalAccuracyArray);
        _horizontalAccuracyArray = nullptr;
    }
    _i2cBus = nullptr;
    if (_latitudeArray)
    {
        free(_latitudeArray);
        _latitudeArray = nullptr;
    }
    if (_longitudeArray)
    {
        free(_longitudeArray);
        _longitudeArray = nullptr;
    }
}

//*********************************************************************
// Initialize the GNSS device
bool R4A_ZED_F9P::begin(Print * display)
{
    int index;
    int retryCount;

    // Attempt to initialize the GNSS receiver
    _online = false;
    for (index = 5; index > 0; index--)
    {
        if (_gnss.begin(*_twoWire, _i2cAddress))
            break;
        delay(100);
    }
    if (index <= 0)
    {
        display->printf("u-blox ZED-F9P not detected at address 0X%02x. Please check wiring.\r\n",
                       _i2cAddress);
        return false;
    }

    //Set the I2C port to output both NMEA and UBX messages
    _gnss.setI2COutput(COM_TYPE_UBX);

    //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
    _gnss.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);

    // Set the differential mode - ambiguities are fixed whenever possible
    _gnss.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);

    _gnss.setNavigationFrequency(1); //Set output in Hz.

    // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
    _gnss.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
    //_gnss.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_DEFAULT); // Uncomment this line to restore the default main talker ID

    // Enable or disable various NMEA sentences over the I2C interface
    _gnss.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual NMEA sentences
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0); // Several of these are on by default so let's disable them
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);

    // Leave only GGA enabled at current navigation rate
    _gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 0);

    if (!_gnss.sendCfgValset()) // Send the configuration VALSET
    {
        display->println(F("NMEA sentence configuration failed!"));
        r4aReportFatalError("u-blox ZED-F9P not detected on I2C bus!");
    }

    // Increase transactions to reduce transfer time
    _gnss.i2cTransactionSize = _i2cTransactionSize;

    _gnss.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA sentences

    //_gnss.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save only the ioPort and sentence settings to NVM

    //_gnss.setNMEAOutputPort(Serial); // Uncomment this line to echo all NMEA data to Serial for debugging

    // Auto-send Valset messages before the buffer is completely full
    _gnss.autoSendCfgValsetAtSpaceRemaining(16);

    // Initialize the callbacks
    _gnss.setAutoPVTcallbackPtr(&r4aZedF9pStorePVTdata);
    _gnss.setAutoHPPOSLLHcallbackPtr(&r4aZedF9pStoreHPdata);

    _online = true;
    return _online;
}

//*********************************************************************
// Compute the mean and standard deviation
double R4A_ZED_F9P::computeMean(double * data,
                                int entries,
                                double * standardDeviation)
{
    double delta;
    int index;
    double mean;
    double sumDeltaSquared;

    // Compute the mean
    mean = 0;
    for (index = 0; index < entries; index++)
        mean += data[index];
    mean /= (double)entries;

    // Compute the standard deviation
    sumDeltaSquared = 0;
    for (index = 0; index < entries; index++)
    {
        delta = data[index] - mean;
        sumDeltaSquared += delta * delta;
    }
    *standardDeviation = sqrt(sumDeltaSquared / (double)entries);

    // Return the mean
    return mean;
}

//*********************************************************************
// Compute point and display point
void R4A_ZED_F9P::computePoint(Print * display)
{
    // Get the horizontal position
    _latLongCount = 50;
    _latLongCountSave = _latLongCount;
    _latitudeArray = (double *)malloc(_latLongCount * sizeof(double));
    _longitudeArray = (double *)malloc(_latLongCount * sizeof(double));
    _horizontalAccuracyArray = (double *)malloc(_latLongCount * sizeof(double));

    // Get the vertical position
    _altitudeCount = _latLongCount;
    _altitudeCountSave = _altitudeCount;
    _altitudeArray = (double *)malloc(_altitudeCount * sizeof(double));

    // Start collection of the location, display the result when done
    _display = display;
    _computePoint = true;
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(Print * display)
{
    displayLocation(_latitude,
                    _longitude,
                    _altitude,
                    _horizontalAccuracy,
                    display);
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(double latitude,
                                  double longitude,
                                  double altitude,
                                  double horizontalAccuracy,
                                  Print * display)
{
    const char * altitudeUnits;
    char buffer[256];
    int correction;
    double hpaInches;
    char hpaUnits;
    int length;
    int tzHours;
    int tzMinutes;
    int tzSeconds;

    // Adjust for the timezone
    correction = _second + r4aTimeZoneSeconds;
    tzSeconds = (correction + 60) % 60;
    tzMinutes = correction / 60;
    correction = _minute + tzMinutes + r4aTimeZoneMinutes;
    tzMinutes = (correction + 60) % 60;
    tzHours = correction / 60;
    correction = _hour + tzHours + r4aTimeZoneHours;
    tzHours = (correction + 24) % 24;
    altitudeUnits = "m";
    hpaUnits = 'm';
    if (r4aZedF9pUnitsFeetInches)
    {
        // Convert the altitude
        altitude = altitude * (double)1000. / R4A_MILLIMETERS_PER_FOOT;
        altitudeUnits = "'";

        // Convert the horizontal position accuracy
        hpaInches = horizontalAccuracy * (double)1000. / R4A_MILLIMETERS_PER_INCH;
        if (hpaInches < 10.)
        {
            hpaUnits = '"';
            horizontalAccuracy = hpaInches;
        }
    }

    // Build the string
    buffer[0] = 0;

    // Display the string
    length = strlen(buffer);
    if (r4aZedF9pDisplayTime)
        sprintf(&buffer[length], "%2d:%02d:%02d", tzHours, tzMinutes, tzSeconds);

    // Display satellites-in-view (SIV)
    length = strlen(buffer);
    if (r4aZedF9pDisplaySiv)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "SIV: %2d", _satellitesInView);
    }

    // Display the location
    length = strlen(buffer);
    if (r4aZedF9pDisplayHpaLatLong)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "HPA: %.3f%c, Lat: %14.9f, Long: %14.9f",
                        horizontalAccuracy, hpaUnits,
                        _latitude, _longitude);
    }

    // Display the altitude
    length = strlen(buffer);
    if (r4aZedF9pDisplayAltitude)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Alt: %9.3f%s", altitude, altitudeUnits);
    }

    // Display the fix type
    length = strlen(buffer);
    if (r4aZedF9pDisplayFixType)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "FT: %d, CS: %d, FR: %d",
                        _fixType, _carrierSolution, _fullyResolved);
    }

    // Display the GPS data
    length = strlen(buffer);
    if (length)
        display->printf("%s\r\n", buffer);
}

//*********************************************************************
// Poll the GNSS using I2C
void R4A_ZED_F9P::i2cPoll()
{
    _gnss.checkUblox(); // Check for the arrival of new data and process it.
}

//*********************************************************************
// Push the RTCM data to the GNSS using I2C
int R4A_ZED_F9P::pushRawData(uint8_t * buffer, int bytes, Print * display)
{

    // I2C: split the data up into packets of i2cTransactionSize
    size_t bytesWrittenTotal = 0;
    while (bytes > 0)
    {
        // Limit bytesToWrite to i2cTransactionSize
        size_t bytesToWrite = bytes;
        if (bytesToWrite > _i2cTransactionSize)
            bytesToWrite = _i2cTransactionSize;

        // Write the bytes
        if (!_i2cBus->write(_i2cAddress,
                            nullptr,        // cmdBuffer
                            0,              // cmdByteCount
                            buffer,         // dataBuffer
                            bytesToWrite,   // dataByteCount
                            display))
        {
            break;
        }

        // Account for the data written
        buffer += bytesToWrite;
        bytes -= bytesToWrite;
        bytesWrittenTotal += bytesToWrite;
    }
    return bytesWrittenTotal;
}

//*********************************************************************
// Store horizontal position data
void R4A_ZED_F9P::storeHPdata(UBX_NAV_HPPOSLLH_data_t * ubxDataStruct)
{
    _horizontalAccuracy = ((float)ubxDataStruct->hAcc) / 10000.0; // Convert hAcc from mm*0.1 to m

    _latitude = ((double)ubxDataStruct->lat) / (double)1.e7;
    _latitude += ((double)ubxDataStruct->latHp) / (double)1.e9;
    _longitude = ((double)ubxDataStruct->lon) / (double)1.e7;
    _longitude += ((double)ubxDataStruct->lonHp) / (double)1.e9;

    // Compute the point location
    if (_computePoint && _latLongCount)
    {
        // Set the array index
        _latLongCount -= 1;

        // Save the values
        _latitudeArray[_latLongCount] = _latitude;
        _longitudeArray[_latLongCount] = _longitude;
        _horizontalAccuracyArray[_latLongCount] = _horizontalAccuracy;
    }
}

//*********************************************************************
// Store vertical position and time data
void R4A_ZED_F9P::storePVTdata(UBX_NAV_PVT_data_t * ubxDataStruct)
{
    _altitude = (double)(ubxDataStruct->height) / (double)1.e3;

    _day = ubxDataStruct->day;
    _month = ubxDataStruct->month;
    _year = ubxDataStruct->year;

    _hour = ubxDataStruct->hour;
    _minute = ubxDataStruct->min;
    _second = ubxDataStruct->sec;
    _nanosecond = ubxDataStruct->nano;
    _millisecond = ceil((ubxDataStruct->iTOW % 1000) / 10.0); // Limit to first two digits

    _satellitesInView = ubxDataStruct->numSV;
    _fixType = ubxDataStruct->fixType; // 0 = no fix, 1 = dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead
                                         // reckoning combined, 5 = time only fix
    _carrierSolution = ubxDataStruct->flags.bits.carrSoln;

    _validDate = ubxDataStruct->valid.bits.validDate;
    _validTime = ubxDataStruct->valid.bits.validTime;
    _confirmedDate = ubxDataStruct->flags2.bits.confirmedDate;
    _confirmedTime = ubxDataStruct->flags2.bits.confirmedTime;
    _fullyResolved = ubxDataStruct->valid.bits.fullyResolved;
    _tAcc = ubxDataStruct->tAcc; // Nanoseconds

    // Compute the point location
    if (_computePoint && _altitudeCount)
    {
        // Set the array index
        _altitudeCount -= 1;

        // Save the value
        _altitudeArray[_altitudeCount] = _altitude;
    }
}

//*********************************************************************
// Process the received NMEA messages
void R4A_ZED_F9P::update(uint32_t currentMsec, Print * display)
{
    static uint32_t lastLocationDisplayMsec;

    _gnss.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    // Display the current location
    if (r4aZedF9pLocationDisplayMsec
        && ((currentMsec - lastLocationDisplayMsec) > r4aZedF9pLocationDisplayMsec))
    {
        lastLocationDisplayMsec = currentMsec;
        displayLocation(&Serial);
    }

    // Display the current point
    if (_computePoint && (!_latLongCount) && (!_altitudeCount))
    {
        double latitudeMean;
        double latitudeStdDev;
        double longitudeMean;
        double longitudeStdDev;
        double horizontalMean;
        double horizontalStdDev;
        double altitudeMean;
        double altitudeStdDev;

        // Stop the data collection
        _computePoint = false;

        // Compute the point
        latitudeMean = computeMean(_latitudeArray,
                                   _latLongCountSave,
                                   &latitudeStdDev);
        longitudeMean = computeMean(_longitudeArray,
                                    _latLongCountSave,
                                    &longitudeStdDev);
        horizontalMean = computeMean(_horizontalAccuracyArray,
                                     _latLongCountSave,
                                     &horizontalStdDev);
        altitudeMean = computeMean(_altitudeArray,
                                   _altitudeCountSave,
                                   &altitudeStdDev);

        // Display the point
        displayLocation(latitudeMean,
                        longitudeMean,
                        altitudeMean,
                        horizontalMean,
                        display);

        // Display the accuracy
        if (display)
        {
            display->printf("%14.9fm: Latitude standard deviation\r\n", latitudeStdDev);
            display->printf("%14.9fm: Longitude standard deviation\r\n", longitudeStdDev);
            display->printf("%14.9fm: Horizontal accuracy standard deviation\r\n", horizontalStdDev);
            display->printf("%14.9fm: Altitude standard deviation\r\n", altitudeStdDev);
        }

        // Free the arrays
        free(_latitudeArray);
        free(_longitudeArray);
        free(_horizontalAccuracyArray);
        free(_altitudeArray);
        _latitudeArray = nullptr;
        _longitudeArray = nullptr;
        _horizontalAccuracyArray = nullptr;
        _altitudeArray = nullptr;
    }
}
