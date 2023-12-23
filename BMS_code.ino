// Include necessary libraries
#include <Wire.h>  // For I2C communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>  // For temperature monitoring

// Define pin assignments for the switches, LEDs, and fault flags
const int switchPin = 44;  // Pin connected to switch control
const int ledPin = 45;     // Built-in LED for status indication
const int potentiometerPin = 46; // Digital pin connected to the variable resistor (potentiometer)
const int currentSensorPin=A1;// Connect to Vout of ACS712
const int faultFlagPin = 48;  // Pin for fault flag signal
const int mosfetPin = 49;  // Digital pin connected to MOSFET's gate
const float cellThresholdVoltage = 3.2;  // Set your threshold voltage here
const int BalanceSwitch1=2;
const int BalanceSwitch2=3;
const int BalanceSwitch3=4;
const int BalanceSwitch4=5;
const int BalanceSwitch5=6;
const int BalanceSwitch6=7;
const int BalanceSwitch7=8;
const int BalanceSwitch8=9;
const int BalanceSwitch9=10;
const int BalanceSwitch10=11;
const int BalanceSwitch11=12;
const int BalanceSwitch12=13;
const int mosPin[]={30,32,34,36,38,40};


// Initialize the BME280 sensor for temperature monitoring
Adafruit_BME280 bme;  // Create a sensor object

// Define constants for voltage thresholds and maximum values
const float minCellVoltage = 2.5;  // Minimum safe cell voltage
const float maxCellVoltage = 4.2;  // Maximum safe cell voltage
const float maxChargingCurrent = 2.0;  // Maximum charging current
const float maxBatteryTemperature = 40.0;  // Maximum safe battery temperature
const float voltageDividerRatio = 0.5;  // Adjust this based on your actual voltage divider configuration
const float calibrationFactor = 1.1;  // Adjust based on calibration

// Define state variables
bool batteryCharging = false;
bool batteryDischarging = false;
bool busFault = false;
bool batteryOverheating = false;

void setup() {
  // Initialize I/O pins
  pinMode(switchPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(faultFlagPin, OUTPUT);
  pinMode(mosfetPin, OUTPUT);
  pinMode(currentSensorPin,OUTPUT);
  
  // Initialize the BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read cell voltages and temperature
  float sum=0;
  // float cellVoltage = readCellVoltage(A0);
  float batteryTemperature = readBatteryTemperature();

  // Check for various fault conditions
  // if (cellVoltage < minCellVoltage) {
  //   stopDischarging();
  //   setFaultFlag();
  //   shutdownSystem();
  // } 
  else if (busFault) {
    attemptBusRecovery();
  } else if (batteryOverheating) {
    stopChargingOrDischarging();
    setFaultFlag();
  // } else if (cellVoltage > maxCellVoltage) {
  //   stopCharging();
  //   setFullFlag();
  // }
   else if (chargingCurrentExceedsMax()) {
    stopCharging();
    setFaultFlag();
  } else {
    // No faults detected, normal operation
    // Implement your charging and discharging logic here
    normalOperation();
  }
}

// Implement functions to read cell voltage, temperature, and control bus

void stopDischarging() {
  digitalWrite(switchPin, LOW);  // Turn off the discharge switch
  batteryDischarging = false;
}

void setFaultFlag() {
  digitalWrite(faultFlagPin, HIGH);  // Set fault flag
}

void shutdownSystem() {
  // Implement a function to gracefully shut down the system
}

void attemptBusRecovery() {
  // Implement bus recovery logic
}

float readCellVoltage(int cellVoltagePin) {
  // Implement voltage reading logic
   // Read the voltage from the analog input pin
  int rawValue = analogRead(cellVoltagePin);

  // Convert the raw ADC value to a voltage in volts
  float voltage = (float)rawValue * (5.0 / 1023.0);  // Assuming a 5V reference voltage

  // Compensate for voltage divider and apply calibration factor
  //Obtain a precise voltage reference source. This can be a voltage reference IC or a calibrated power supply that provides a known and stable voltage
  //Connect the reference voltage to the input of your voltage divider circuit, just like you would with a cell voltage.
  //Measure the raw ADC readings corresponding to the known reference voltage. Record these readings.
  //the value that is obtained from analog pin is proportional to the voltage at that point, to convert it accurately we use voltage divider ratio
  float actualVoltage = measuredVoltage / voltageDividerRatio * calibrationFactor; //calibrationfactor = (expected voltage) / (measured voltage)

  return actualVoltage;  // Replace with actual voltage reading
  // Add a delay to control the sampling rate (adjust as needed)
  delay(1000);  // 1-second delay
 
}

float readBatteryTemperature() {
  // Implement temperature reading logic
  return bme.readTemperature();  // Replace with actual temperature reading
}

bool chargingCurrentExceedsMax() {
  // Implement current monitoring logic
  return false;  // Replace with actual current monitoring
}

void normalOperation() {
  // Implement your charging and discharging logic
  // Ensure safe and efficient battery management
  float sum=0;
  for (int cell = 1; cell <= 6; cell++) {
      // Calculate the index for the current cell
      int cellVoltagePin = A0 + cell - 1;
      float cellVoltage = readCellVoltage(cellVoltagePin);
      
      if(cellVoltage>maxCellVoltage)
      {
        stopCharging(cell);
      }

      if (cellVoltage < minCellVoltage) 
      {
        stopDischarging();
        shutdownSystem();
      }

      // Read the voltage of the current cell
      sum + = cellVoltage;
    }
  if(sum<12.8)
  {
    digitalWrite(mosfetPin, HIGH);  // Turn on the MOSFET
    for (int cell = 1; cell <= 6; cell++) {
      // Calculate the index for the current cell
      int cellVoltagePin = A0 + cell - 1;

      // Read the voltage of the current cell
      float cellVoltage = readCellVoltage(cellVoltagePin);

      // Check if the cell needs charging
      if (cellVoltage < cellThresholdVoltage) {
        // Charge the cell based on its voltage
        chargeCell(cellVoltage, cell);
      }
    }
  }
    else if(sum>=23.4) // 3.9 * 6 = 23.4
    {
      analogWrite(BalanceSwitch1,153); // 153 is 60 percent duty cycle
      analogWrite(BalanceSwitch2,102); // 102 is 40 percent duty cycle
      analogWrite(BalanceSwitch3,153);
      analogWrite(BalanceSwitch4,102);
      analogWrite(BalanceSwitch5,153);
      analogWrite(BalanceSwitch6,102);
      analogWrite(BalanceSwitch7,153);
      analogWrite(BalanceSwitch8,102);
      analogWrite(BalanceSwitch9,153);
      analogWrite(BalanceSwitch10,102);
      analogWrite(BalanceSwitch11,153);
      analogWrite(BalanceSwitch12,102);

    }
    
  //   //read curret sensor data
  //   unsigned int x=0;
  //   float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
 
  //   for (int x = 0; x < 150; x++){ //Get 150 samples
  //     AcsValue = analogRead(A1);     //Read current sensor values  
  //     Samples = Samples + AcsValue;  //Add samples together
  //     delay (3); // let ADC settle before next sample 3ms
  //   }
  //   AvgAcs=Samples/150.0;//Taking Average of Samples
 
  //   AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.100;
  //   delay(50);
  }
void chargeCell(float cellVoltage, int cellNumber) {
    if(cellVoltage<2.5)
    {
      digitalWrite(mosPin[cellNumber-1],HIGH);
      int resistorValue = map(0.2/(29-cellVoltage), -2.0, 2.0, 0, 255); // Adjust the mapping range as needed where  29V is output from converter
       // Set the variable resistor value
       analogWrite(potentiometerPin, resistorValue);
    }
    else if(cellVoltage>2.5 && cellVoltage<3.2)
    {
      digitalWrite(mosPin[cellNumber-1],HIGH);
      int resistorValue = map(5/(29-cellVoltage), -2.0, 2.0, 0, 255); // Adjust the mapping range as needed where 29V is output from converter and 5A is Imax of converter
       // Set the variable resistor value
       analogWrite(potentiometerPin, resistorValue);
    }
  }
 

void setFullFlag() {
  // Implement logic to set the 'full' flag
}

void stopCharging(int cell) {
  digitalWrite(mosPin[cell-1],LOW); // turns off the corresponding mosfet connected to the converter and battery cell.
}

void stopChargingOrDischarging() {
  // Implement logic to stop both charging and discharging
}
