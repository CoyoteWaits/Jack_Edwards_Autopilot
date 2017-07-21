#if Compass == 1
/*************************************************************/
void BNO_RestoreCal()
{
//  Get and restore BNO Calibration offsets  

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
       // Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
       lcd.print("NO BNO CAL DATA");
       lcd.setCursor(0,1); lcd.print("DO MANUAL CAL");
        delay(2000);
    }
    else
    {
        //Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

      //  displaySensorOffsets(calibrationData);

        //Serial.println("\n\nRestoring Calibration data to the BNO055...");
        lcd.begin(20,4);
        lcd.setCursor(0,0);
        lcd.print ("LOADING BNO DATA");
        bno.setSensorOffsets(calibrationData);

        //Serial.println("\n\nCalibration data loaded into BNO055");
        lcd.setCursor(0,1);
        lcd.print("BNO CAL DATA LOADED");
        foundCalib = true;
        delay(2000);
    }
//  get and restore BNO calibration


}  // end BNO Restore Cal
/*************************************************************/
void BNO_SaveCal()
{
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    //Serial.println("\n\nStoring calibration data to EEPROM...");
    lcd.clear();
    lcd.print("Storing Calibration");
    eeAddress = 0;
    sensor_t sensor;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    //Serial.println("Data stored to EEPROM.");
    DataStored = true;
    Serial.println("\n--------------------------------\n");
    delay(2000);

}  //  end BNO_SaveCal
/********************************************************/

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/****************************************************************/
#endif

