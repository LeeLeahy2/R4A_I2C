/**********************************************************************
  ZED_F9P.cpp

  Support for the u-blox ZED-F9P GNSS receiver
**********************************************************************/

#include <R4A_I2C.h>

//****************************************
// Globals - Parameters
//****************************************

bool r4aZedF9pDisplayAltitude;          // Display the altitude
bool r4aZedF9pDisplayAltitudeStdDev;    // Display the altitude standard deviation
bool r4aZedF9pDisplayFixType;           // Display the fix type
bool r4aZedF9pDisplayHorizAcc;          // Display the horizontal accuracy
bool r4aZedF9pDisplayHorizAccStdDev;    // Display the horizontal accuracy standard deviation
bool r4aZedF9pDisplayLatitude;          // Display the latitude
bool r4aZedF9pDisplayLatitudeStdDev;    // Display the latitude standard deviation
bool r4aZedF9pDisplayLongitude;         // Display the longitude
bool r4aZedF9pDisplayLongitudeStdDev;   // Display the longitude standard deviation
bool r4aZedF9pDisplaySiv;               // Display satellites-in-view
bool r4aZedF9pDisplayTime;              // Display time data

uint32_t r4aZedF9pLocationDisplayMsec = 1000; // 0 = Off, Interval to display the location
uint32_t r4aZedF9pPollMsec = 100;       // I2C polling interval for the GNSS receiver
bool r4aZedF9pUnitsFeetInches;          // Display in feet and inches .vs. meters

R4A_ZED_F9P * r4aZedF9p;

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

    // Save the object address
    r4aZedF9p = this;

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
// Start collecting data for a point
bool R4A_ZED_F9P::collectData(int count, const char * comment, Print * display)
{
    // Verify that the collection is available
    if (_latLongCount)
        return false;

    // Save the comment
    _comment = comment;

    // Get the horizontal position
    _latLongCount = count;
    _latLongCountSave = _latLongCount;
    _latitudeArray = (double *)malloc(_latLongCount * sizeof(double));
    _longitudeArray = (double *)malloc(_latLongCount * sizeof(double));
    _horizontalAccuracyArray = (double *)malloc(_latLongCount * sizeof(double));

    // Get the vertical position
    _altitudeCount = _latLongCount;
    _altitudeCountSave = _altitudeCount;
    _altitudeArray = (double *)malloc(_altitudeCount * sizeof(double));

    // Set the display
    _display = display;
    return true;
}

//*********************************************************************
// Compute and display a point
void R4A_ZED_F9P::computePoint(R4A_DISPLAY_ROUTINE routine,
                               intptr_t parameter,
                               int count,
                               const char * comment,
                               Print * display)
{
    bool idle;

    // Start collection of the location, display the result when done
    idle = collectData(count, comment, display);
    if (idle)
    {
        _displayRoutine = routine;
        _displayParameter = parameter;
    }
    else if (display)
        display->printf("ERROR: Data collection is busy!\r\n");
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(const char * comment, Print * display)
{
    displayLocation(comment,
                    _latitude,
                    0,
                    _longitude,
                    0,
                    _altitude,
                    0,
                    _horizontalAccuracy,
                    0,
                    _satellitesInView,
                    r4aZedF9pUnitsFeetInches,
                    r4aZedF9pDisplayTime,
                    r4aZedF9pDisplaySiv,
                    r4aZedF9pDisplayLatitude,
                    false,
                    r4aZedF9pDisplayLongitude,
                    false,
                    r4aZedF9pDisplayHorizAcc,
                    false,
                    r4aZedF9pDisplayAltitude,
                    false,
                    r4aZedF9pDisplayFixType,
                    display);
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(const char * comment,
                                  bool unitsFeetInches,
                                  bool displayTime,
                                  bool displaySiv,
                                  bool displayLatitude,
                                  bool displayLongitude,
                                  bool displayHorizAcc,
                                  bool displayAltitude,
                                  bool displayFixType,
                                  Print * display)
{
    displayLocation(comment,
                    _latitude,
                    0,
                    _longitude,
                    0,
                    _altitude,
                    0,
                    _horizontalAccuracy,
                    0,
                    _satellitesInView,
                    unitsFeetInches,
                    displayTime,
                    displaySiv,
                    displayLatitude,
                    false,
                    displayLongitude,
                    false,
                    displayHorizAcc,
                    false,
                    displayAltitude,
                    false,
                    displayFixType,
                    display);
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(const char * comment,
                                  double latitude,
                                  double longitude,
                                  double altitude,
                                  double horizontalAccuracy,
                                  uint8_t satellitesInView,
                                  Print * display)
{
    displayLocation(comment,
                    latitude,
                    0,
                    longitude,
                    0,
                    altitude,
                    0,
                    horizontalAccuracy,
                    0,
                    satellitesInView,
                    r4aZedF9pUnitsFeetInches,
                    r4aZedF9pDisplayTime,
                    r4aZedF9pDisplaySiv,
                    r4aZedF9pDisplayLatitude,
                    false,
                    r4aZedF9pDisplayLongitude,
                    false,
                    r4aZedF9pDisplayHorizAcc,
                    false,
                    r4aZedF9pDisplayAltitude,
                    false,
                    r4aZedF9pDisplayFixType,
                    display);
}

//*********************************************************************
// Display the location
void R4A_ZED_F9P::displayLocation(const char * comment,
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
    if (unitsFeetInches)
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

    // Add the comment
    length = strlen(buffer);
    if (comment)
        sprintf(&buffer[length], "%s", comment);

    // Display the string
    length = strlen(buffer);
    if (displayTime)
        sprintf(&buffer[length], "%2d:%02d:%02d", tzHours, tzMinutes, tzSeconds);

    // Display satellites-in-view (SIV)
    length = strlen(buffer);
    if (displaySiv)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "SIV: %2d", satellitesInView);
    }

    // Display the horizontal accuracy
    length = strlen(buffer);
    if (displayHorizAcc)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "HPA: %.3f%c", horizontalAccuracy, hpaUnits);
    }

    // Display the horizontal accuracy standard deviation
    length = strlen(buffer);
    if (displayHorizAccStdDev)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Std Dev: %14.9f", horizontalAccuracyStdDev);
    }


    // Display the latitude
    length = strlen(buffer);
    if (displayLatitude)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Lat: %14.9f", latitude);
    }

    // Display the latitude standard deviation
    length = strlen(buffer);
    if (displayLatStdDev)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Std Dev: %14.9f", latitudeStdDev);
    }

    // Display the longitude
    length = strlen(buffer);
    if (displayLongitude)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Long: %14.9f", longitude);
    }

    // Display the longitude standard deviation
    length = strlen(buffer);
    if (displayLongStdDev)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Std Dev: %14.9f", longitudeStdDev);
    }

    // Display the altitude
    length = strlen(buffer);
    if (displayAltitude)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Alt: %9.3f%s", altitude, altitudeUnits);
    }

    // Display the altitude standard deviation
    length = strlen(buffer);
    if (displayAltitudeStdDev)
    {
        if (length)
        {
            strcat(buffer, "  ");
            length += 2;
        }
        sprintf(&buffer[length], "Std Dev: %14.9f", longitudeStdDev);
    }

    // Display the fix type
    length = strlen(buffer);
    if (displayFixType)
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
    if (length && display)
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
    if (_latLongCount)
    {
        // Set the array index
        _latLongCount -= 1;

        // Save the values
        _latitudeArray[_latLongCount] = _latitude;
        _longitudeArray[_latLongCount] = _longitude;
        _horizontalAccuracyArray[_latLongCount] = _horizontalAccuracy;

        // Compute the point when the data collection is done
        if (_latLongCount == 0)
        {
            _latitudeMean = computeMean(_latitudeArray,
                                        _latLongCountSave,
                                        &_latitudeStdDev);
            _longitudeMean = computeMean(_longitudeArray,
                                         _latLongCountSave,
                                         &_longitudeStdDev);
            _horizontalMean = computeMean(_horizontalAccuracyArray,
                                          _latLongCountSave,
                                          &_horizontalStdDev);

            // Free the arrays
            free(_latitudeArray);
            free(_longitudeArray);
            free(_horizontalAccuracyArray);
            _latitudeArray = nullptr;
            _longitudeArray = nullptr;
            _horizontalAccuracyArray = nullptr;
        }
    }
    _hpDataAvailable = true;
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
    if (_altitudeCount)
    {
        // Set the array index
        _altitudeCount -= 1;

        // Save the value
        _altitudeArray[_altitudeCount] = _altitude;

        // Compute the point when the data collection is done
        if (_altitudeCount == 0)
        {
            _altitudeMean = computeMean(_altitudeArray,
                                        _altitudeCountSave,
                                        &_altitudeStdDev);

            // Free the array
            free(_altitudeArray);
            _altitudeArray = nullptr;
        }
    }
}

//*********************************************************************
// Process the received NMEA messages
void R4A_ZED_F9P::update(uint32_t currentMsec, const char * comment, Print * display)
{
    static uint32_t lastLocationDisplayMsec;
    bool lastPoint;

    _gnss.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    // Display the current location
    if (r4aZedF9pLocationDisplayMsec && _hpDataAvailable
        && ((currentMsec - lastLocationDisplayMsec) >= r4aZedF9pLocationDisplayMsec))
    {
        lastLocationDisplayMsec = currentMsec;
        displayLocation(comment, &Serial);
        _hpDataAvailable = false;
    }

    // Determine if this is the last point in the array
    lastPoint = (!_latLongCount) && (!_altitudeCount);
    if (!lastPoint)
        return;

    // Display the computed point
    if (_displayRoutine)
    {
        // Display the computed point
        _displayRoutine(_displayParameter,
                        _comment,
                        _latitudeMean,
                        _latitudeStdDev,
                        _longitudeMean,
                        _longitudeStdDev,
                        _altitudeMean,
                        _altitudeStdDev,
                        _horizontalMean,
                        _horizontalStdDev,
                        _satellitesInView,
                        _display);

        // Stop the data collection
        _displayParameter = 0;
        _displayRoutine = nullptr;
        _comment = nullptr;
        _display = nullptr;
    }
}
