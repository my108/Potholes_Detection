#include <SD.h>

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance; 
long fsrForce;                                                  

int RXPin = 4;
int TXPin = 3;

TinyGPSPlus gps;
SoftwareSerial SerialGPS(RXPin, TXPin);
int chipSelect = 4; //chip select pin for the MicroSD Card Adapter
File file;
void setup()
{
  Serial.begin(9600);
  SerialGPS.begin(9600);
  pinMode(chipSelect, OUTPUT); // chip select pin must be set to OUTPUT mode
  if (!SD.begin(chipSelect)) { // Initialize SD card
    Serial.println("Could not initialize SD card."); // if return value is false, something went wrong.
  }
}

void loop()
{
  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read()))
      showData();
      
  
      
  if (millis() > 10000 && gps.charsProcessed() < 10)
  {
    Serial.println("GPS NOT DETECTED!");
    while(true);
  }
  fsrReading = analogRead(fsrPin);  
  
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  Serial.print("Voltage reading in mV = ");
  Serial.println(fsrVoltage);  
 
  if (fsrVoltage == 0) {
    Serial.println("No pressure");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    
 
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    
 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);      
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);            
    }
  }
  
}

void showData()
{
  if (gps.location.isValid() and fsrForce>5)
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    
 }
   
 }
void savedata()
{  
  if (gps.location.isValid() and fsrForce>5){
   file = SD.open("file.txt", FILE_WRITE); // open "file.txt" to write data
    if (file) {
      file.print("Latitude: "); // write number to file
      file.println(gps.location.lat(), 6);
      file.print("Longitude: ");
      file.println(gps.location.lng(), 6);
    
    
      file.close(); // close file
    
  } else {
    Serial.println("Could not open file (writing).");
  }
  }
}
