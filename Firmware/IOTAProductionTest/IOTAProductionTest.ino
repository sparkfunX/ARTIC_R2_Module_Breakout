/*
  IOTA Production Test Code
  
  ** Requires the IOTA Test Jig and the Artemis Thing Plus **
  
  Written by Paul Clark (PaulZC)
  20th March 2021
  
  ** This code has only been tested using v1.2.1 of the Apollo3 Boards **
  ** Set the Board to "SparkFun Artemis Thing Plus" **
  
  Connect an antenna, press the IOTA gently onto the Test Jig spring pins, then click "GO" to begin.
  
  Helpful messages will appear in the Serial Monitor (115200 Baud) as the test progresses.
  
  You will need to install the SparkFun ARGOS ARTIC R2 Arduino Library.
  Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
  
  License: please see the license file at:
  https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/LICENSE.md
  
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17236
  
  Hardware Connections:
  This example assumes the ARTIC Breakout has been mounted on a SparkFun Thing Plus - Artemis:
  https://www.sparkfun.com/products/15574
  CS_Pin = A5 (D24)
  GAIN8_Pin = D3
  BOOT_Pin = D4
  INT1_Pin = D5
  INT2_Pin = D6
  RESET_Pin = D7
  IOTA_PWR_EN_Pin = D8
  (SPI COPI = D11)
  (SPI CIPO = D12)
  (SPI SCK = D13)
  
*/

// Uncomment the "#define noFail" to allow the test to continue even though it has failed - USE WITH CAUTION
//#define noFail

#define testTimeout 10000 // Timeout for Tests 5 & 7 in millis
unsigned long testStart;

// This is the example used in A4-SS-TER-SP-0079-CNES
// The complete over-air data stream, including sync pattern and length, should be: 0xAC5353DC651CECA2F6E328E76517B719473B28BD followed by 0b1
const uint32_t PLATFORM_ID = 0x01234567;

const uint32_t tcxoWarmupTime = 2; // Define the TCXO warmup time - should really be 10 seconds but time is money...

#include <SPI.h>

#include "SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ARGOS_ARTIC_R2
ARTIC_R2 myARTIC;

// Pin assignments for the SparkFun Thing Plus - Artemis
// (Change these if required)
int CS_Pin = 24;
int GAIN8_Pin = 3; // Optional. Set to -1 if you don't want to control the gain. The library defaults to maximum power.
int BOOT_Pin = 4;
int INT1_Pin = 5;
int INT2_Pin = 6;
int RESET_Pin = 7;
int IOTA_PWR_EN_Pin = 8;

// Test Jig Pins
int FAIL = 16; // FAIL LED on SDA / D16
int PASS = 17; // PASS LED on SCL / D17
int GO = A4 ; // GO switch on A4 / D
int A_3V0 = A3; // RF Amp 3.0V Rail / 2
int VDD = A1; // IOTA 3.3V Rail / 2
int A_1V8 = A0; // ARTIC 1.8V Rail / 2

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to jump between any of the steps
enum {
  wait_for_GO,          // Wait for the user to press the GO button (or hit Enter)
  antenna_check,        // Flash the LEDs and ask user to confirm that the antenna is attached
  test_VDD,             // Enable 3.3V power for the ARTIC. Check that VDD (3.3V), 3.0V and the 1.8V rails are within tolerance
  check_ARTIC,          // Power on the ARTIC (again) and check the firmware version and checksum
  configure_ARTIC,      // Configure the ARTIC (set the satellite detection timeout and TX mode)
  ARTIC_RX,             // Wait for satellite detection to time out
  ARTIC_TX,             // Start the ARTIC TX
  wait_for_ARTIC_TX,    // Wait for the ARTIC to transmit
  tests_complete        // Tests are complete. Turn on the PASS or FAIL LED
} loop_steps;
int loop_step = wait_for_GO; // Make sure loop_step is set to wait_for_GO

//Globals
bool testFailed = false; // Flag to show if the test failed (but might have continued anyway if noFail is defined)

void setup()
{
  // Configure the I/O pins
  
  pinMode(IOTA_PWR_EN_Pin, OUTPUT);

  pinMode(FAIL, OUTPUT); // Configure the FAIL LED
  pinMode(PASS, OUTPUT); // Configure the PASS LED
  
  pinMode(GO, INPUT_PULLUP); // Configure the GO button

  pinMode(INT1_Pin, INPUT);
  pinMode(INT2_Pin, INPUT);

  pinMode(GAIN8_Pin, OUTPUT);
  pinMode(BOOT_Pin, OUTPUT);
  pinMode(RESET_Pin, OUTPUT);

  pinsOff();

  //Set ADC resolution to 14 bit
  analogReadResolution(14);

  // Start the console serial port
  Serial.begin(115200);
}

void loop()
{
  // loop is one large switch/case that controls the sequencing of the code
  switch (loop_step) {

    // ************************************************************************************************
    // Wait for the GO button to be pressed or a Newline to be received
    case wait_for_GO:
    {
      testFailed = false; // Reset testFailed
      
      delay(100);
      Serial.println();
      Serial.println(F("IOTA Production Test"));
      Serial.println();
    
      //Wait for the GO button to be pressed or a Newline to be received
      Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
      Serial.println(F("and that the line ending is set to Newline."));
      Serial.println(F("Then press GO or click Send to start the test."));
      Serial.println();
      
      //empty the serial buffer
      while(Serial.available() > 0)
        Serial.read();
    
      while((Serial.available() == 0) && (!GOclick())) // Wait for serial data or a GO click
        ;
  
      delay(100);
  
      //empty the serial buffer
      while(Serial.available() > 0)
        Serial.read();
    
      Serial.println();
      Serial.println();
      Serial.println(F("Please check that there is an antenna connected to the IOTA Test Jig."));
      Serial.println(F("Press GO or click Send to confirm."));
      Serial.println();

      loop_step = antenna_check; // Move on
    }
    break;

    // ************************************************************************************************
    // Flash the LEDs and ask user to confirm that the antenna is attached
    case antenna_check:
    {
      if ((millis() % 200) < 100)
      {
        digitalWrite(PASS, HIGH);
        digitalWrite(FAIL, LOW);
      }
      else
      {
        digitalWrite(PASS, LOW);
        digitalWrite(FAIL, HIGH);
      }
      
      if((Serial.available()) || (GOclick())) // Wait for serial data or a GO click
      {
        delay(100);

        //empty the serial buffer
        while(Serial.available() > 0)
          Serial.read();
      
        digitalWrite(FAIL, LOW); // Turn the LEDs off
        digitalWrite(PASS, LOW);
  
        loop_step = test_VDD; // Move on
      }
    }
    break;

    // ************************************************************************************************
    // Test 1 - Enable 3.3V power for the ARTIC. Check that VDD (3.3V) rail, 3.0V rail and the 1.8V rail are within tolerance
    case test_VDD:
    {
      digitalWrite(IOTA_PWR_EN_Pin, HIGH); // Enable power

      delay(1000); // Wait - give the operator time to check the PWR LED
      
      float voltsVDD = ((float)analogRead(VDD)) * 2.0 * 1.06 * 2.0 / 16384.0; // Divide-by-2 * fiddle factor * 2.0V ADC range / 2^14
      
      Serial.print(F("Test 1 Step 1: The ARTIC VDD (3.3V) power rail is: "));
      Serial.print(voltsVDD, 2);
      Serial.println(F("V"));
    
      if (voltsVDD < 3.2)
      {
        Serial.println(F("Test 1 Step 1: ARTIC VDD is too low!"));
        fail();
      }
      if (voltsVDD > 3.4)
      {
        Serial.println(F("Test 1 Step 1: ARTIC VDD is too high!"));
        fail();
      }

      float volts1V8 = ((float)analogRead(A_1V8)) * 2.0 * 1.06 * 2.0 / 16384.0; // Divide-by-2 * fiddle factor * 2.0V ADC range / 2^14
      
      Serial.print(F("Test 1 Step 2: The ARTIC 1.8V power rail is: "));
      Serial.print(volts1V8, 2);
      Serial.println(F("V"));
    
      if (volts1V8 < 1.7)
      {
        Serial.println(F("Test 1 Step 2: ARTIC 1.8V rail is too low!"));
        fail();
      }
      if (volts1V8 > 1.9)
      {
        Serial.println(F("Test 1 Step 2: ARTIC 1.8V rail is too high!"));
        fail();
      }

      float volts3V0 = ((float)analogRead(A_3V0)) * 2.0 * 1.06 * 2.0 / 16384.0; // Divide-by-2 * fiddle factor * 2.0V ADC range / 2^14
      
      Serial.print(F("Test 1 Step 3: The RF Amp 3.0V power rail is: "));
      Serial.print(volts3V0, 2);
      Serial.println(F("V"));
    
      if (volts3V0 < 2.9)
      {
        Serial.println(F("Test 1 Step 3: RF Amp 3.0V rail is too low!"));
        fail();
      }
      if (volts3V0 > 3.1)
      {
        Serial.println(F("Test 1 Step 3: RF Amp 3.0V rail is too high!"));
        fail();
      }

      if (loop_step != tests_complete)
        loop_step = check_ARTIC; // Move on
    }
    break;
    
    // ************************************************************************************************
    // Test 3 - Check the ARTIC firmware version and checksum
    case check_ARTIC:
    {
      SPI.begin();
    
      // Uncomment the next line to enable the helpful debug messages
      //myARTIC.enableDebugging(); // Enable debug messages to Serial
    
      Serial.println(F("Test 3 Step 1: Starting the ARTIC R2..."));
    
      // Begin the ARTIC: enable power and boot from flash
      if (myARTIC.beginIOTA(CS_Pin, RESET_Pin, BOOT_Pin, IOTA_PWR_EN_Pin, INT1_Pin, INT2_Pin, GAIN8_Pin) == false)
      {
        Serial.println("Test 3 Step 1: ARTIC R2 not detected!");
        fail();
      }
      
      char buffer[9]; // Buffer to store the firmware version (8 bytes + NULL)
    
      myARTIC.readFirmwareVersion(&buffer[0]); // Read the firmware version from PMEM
    
      Serial.print(F("Test 3 Step 2: ARTIC R2 firmware version is: "));
      Serial.println(buffer);

      if ((buffer[5] != '0') || (buffer[6] != '0') || (buffer[7] != (ARTIC_R2_FIRMWARE_VERSION + '0')))
      {
        Serial.print(F("Test 3 Step 2: Firmware version does not match!"));
        fail();      
      }

      // Read the checksum words
      uint32_t PMEM_CRC, XMEM_CRC, YMEM_CRC;
      myARTIC.readMemoryCRC(&PMEM_CRC, &XMEM_CRC, &YMEM_CRC);
    
      // Check that the checksums match
      // Unfortunately I have had to hard-code the checksums as ARTIC_R2_Firmware_nMEM.h are not included when using flash
      if ((PMEM_CRC != ARTIC_R2_PMEM_CHECKSUM) || (XMEM_CRC != ARTIC_R2_XMEM_CHECKSUM) || (YMEM_CRC != ARTIC_R2_YMEM_CHECKSUM))
      {
        Serial.print(F("Test 3 Step 3: Firmware checksum fail!"));
        fail();
      }

      if (loop_step != tests_complete)
        loop_step = configure_ARTIC; // Move on
    }
    break;

    // ************************************************************************************************
    // Test 4 - Configure the ARTIC (set the satellite detection timeout and TX mode)
    case configure_ARTIC:
    {
      // Set the TCXO voltage to 1.8V and autoDisable to 1
      if (myARTIC.setTCXOControl(1.8, true) == false)
      {
        Serial.println(F("Test 4 Step 1: setTCXOControl failed!"));
        fail();
      }

      // Set the TCXO warm-up time
      if (myARTIC.setTCXOWarmupTime(tcxoWarmupTime) == false)
      {
        Serial.println(F("Test 4 Step 2: setTCXOWarmupTime failed!"));
        fail();
      }

      // Set the satellite detection timeout to 2 seconds
      if (myARTIC.setSatelliteDetectionTimeout(2) == false)
      {
        Serial.println(F("Test 4 Step 3: setSatelliteDetectionTimeout failed!"));
        fail();
      }

      // Set the TX mode to ARGOS 4 VLD
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendConfigurationCommand(CONFIG_CMD_SET_ARGOS_4_PTT_VLD_TX_MODE);
      if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
      {
        Serial.println(F("Test 4 Step 4: sendConfigurationCommand failed!"));
        fail();
      }

      // Set the ARGOS 4 TX frequency to 401.630 MHz
      // From A4-SS-TER-SP-0079-CNES:
      // The transmission frequency for PTT-VLD-A4 platforms shall be set between 399.91 MHz to 401.68 MHz.
      // Due to frequency regulations, the frequency ranges [400.05 MHz to 401.0 MHz] and [401.2 MHz to 401.3 MHz] are forbidden for VLD-A4 transmissions.
      if (myARTIC.setARGOS4TxFrequency(401.630) == false)
      {
        Serial.println(F("Test 4 Step 5: setARGOS4TxFrequency failed!"));
        fail();
      }

      // Print the TX frequency
      float tx4freq = myARTIC.getARGOS4TxFrequency();
      Serial.print(F("Test 4: The ARGOS 4 TX Frequency is "));
      Serial.print(tx4freq, 3);
      Serial.println(F(" MHz."));

      // Uncomment the next line if you want to attenuate the transmit power by 8dB
      myARTIC.attenuateTXgain(true);

      // Start satellite detection
      // The ARTIC will start looking for a satellite for the specified amount of time.
      // If no satellite is detected, INT 2 will be set with the ‘SATELLITE_TIMEOUT’ flag.
      // If a satellite was detected, by receiving 5 consecutive 0x7E flags, INT 1 will be set with the ‘RX_SATELLITE_DETECTED’ flag.
      // sendMCUinstruction returns an ARTIC_R2_MCU_Command_Result
      // sendMCUinstruction will return ARTIC_R2_MCU_COMMAND_ACCEPTED if the instruction was accepted
      result = myARTIC.sendMCUinstruction(INST_SATELLITE_DETECTION);
    
      if ((result == ARTIC_R2_MCU_COMMAND_REJECTED) || (result == ARTIC_R2_MCU_COMMAND_OVERFLOW))
      {
        Serial.println(F("Test 4 Step 6: sendMCUinstruction failed!"));
        fail();
      }

      delay(100);

      if (digitalRead(INT2_Pin) != LOW)
      {
        Serial.println("Test 4 Step 7: INT2_Pin is not low!");
        fail();
      }      

      testStart = millis();

      if (loop_step != tests_complete)
        loop_step = ARTIC_RX; // Move on      
    }
    break;

    // ************************************************************************************************
    // Test 5 - Wait for ARTIC RX to timeout
    case ARTIC_RX:
    {
      delay(1000); // Check the status every second

      // Read and print the ARTIC R2 status register
      ARTIC_R2_Firmware_Status status;
      myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
      Serial.println(F("Test 5 Step 1: ARTIC R2 Firmware Status:"));
      myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial

      // Note: satellite detection does not time out once a satellite has been detected. The firmware goes IDLE instead.
    
      // Note: With ARTIC006 firmware: when satellite detection times out and INT2 goes high, INT1 goes high too! Confusing...
      // We should only believe INT1 if INT2 is low.
    
      if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // Check the interrupt 2 flag. This will go high if satellite detection times out
      {
        Serial.println(F("Test 5 Step 2: INT2 pin is high. Satellite detection has timed out!"));
      }
      else if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when a satellite is detected
      {
        Serial.println(F("Test 5 Step 3: INT1 pin is high. Satellite detected!"));
        fail(); // This should be impossible indoors?!
#ifdef noFail
        loop_step = ARTIC_TX; // Move on
#else
        loop_step = tests_complete; // We're done
#endif
      }

      // Read and print the instruction progress
      ARTIC_R2_MCU_Instruction_Progress progress;
      // checkMCUinstructionProgress will return true if the instruction is complete
      boolean instructionComplete = myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress
      Serial.println(F("Test 5 Step 1: ARTIC R2 instruction progress:"));
      myARTIC.printInstructionProgress(progress); // Pretty-print the progress to Serial

      if (instructionComplete)
      {
        Serial.println(F("Test 5: RX is complete!"));

        if (digitalRead(INT2_Pin) != HIGH)
        {
          Serial.println("Test 5 Step 4: INT2_Pin is not high!");
          fail();
#ifdef noFail
          loop_step = ARTIC_TX; // Move on
#else
          loop_step = tests_complete; // We're done
#endif
        }

        // Clear INT2
        if (myARTIC.clearInterrupts(1) == false)
        {
          Serial.println(F("Test 5 Step 5: clearInterrupts failed!"));
          fail();
#ifdef noFail
          loop_step = ARTIC_TX; // Move on
#else
          loop_step = tests_complete; // We're done
#endif
        }

        if (loop_step == ARTIC_RX)
          loop_step = ARTIC_TX; // Move on
      }

      if (millis() > (testStart + testTimeout))
      {
        Serial.println(F("Test 5 Step 6: timeout!"));
        fail();
#ifdef noFail
        loop_step = ARTIC_TX; // Move on
#else
        loop_step = tests_complete; // We're done
#endif        
      }
    }
    break;

    // ************************************************************************************************
    // Test 6 - Start the ARTIC TX
    case ARTIC_TX:
    {
      // Configure the Tx payload for ARGOS 4 VLD using the platform ID
      if (myARTIC.setPayloadARGOS4VLDLong(PLATFORM_ID, PLATFORM_ID, PLATFORM_ID) == false)
      {
        Serial.println(F("Test 6 Step 1: setPayloadARGOS4VLDLatLon failed!"));
        fail();
      }

      // Tell the ARTIC to do its thing!
      ARTIC_R2_MCU_Command_Result result = myARTIC.sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE);
      if (result != ARTIC_R2_MCU_COMMAND_ACCEPTED)
      {
        Serial.println("Test 6 Step 2: sendMCUinstruction(INST_TRANSMIT_ONE_PACKAGE_AND_GO_IDLE) failed!");
        fail();
      }

      testStart = millis();

      delay(100);

      if (digitalRead(INT1_Pin) != LOW)
      {
        Serial.println("Test 6 Step 3: INT1_Pin is not low!");
        fail();
      }

      if (loop_step != tests_complete)
        loop_step = wait_for_ARTIC_TX; // Move on

      Serial.println("Test 6: Complete");
    }
    break;

    // ************************************************************************************************
    // Test 7 - Start the ARTIC in Transmit One Package And Go Idle mode
    case wait_for_ARTIC_TX:
    {
      delay(1000); // Check the status every second

      // Read and print the ARTIC R2 status register
      ARTIC_R2_Firmware_Status status;
      myARTIC.readStatusRegister(&status); // Read the ARTIC R2 status register
      Serial.println(F("Test 7 Step 1: ARTIC R2 Firmware Status:"));
      myARTIC.printFirmwareStatus(status); // Pretty-print the firmware status to Serial

      if (status.STATUS_REGISTER_BITS.DSP2MCU_INT1) // Check the interrupt 1 flag. This will go high when TX is finished
      {
        Serial.println(F("Test 7 Step 2: INT1 pin is high. TX is finished (or MCU is in IDLE_STATE)!"));
      }

      if (status.STATUS_REGISTER_BITS.DSP2MCU_INT2) // Check the interrupt 2 flag. This will go high when if the message was invalid
      {
        Serial.println(F("Test 7 Step 3: INT2 pin is high. TX message was invalid! (Something really bad must have happened...)"));
        fail();
        loop_step = tests_complete; // We're done
      }

      // Read and print the instruction progress
      ARTIC_R2_MCU_Instruction_Progress progress;
      // checkMCUinstructionProgress will return true if the instruction is complete
      boolean instructionComplete = myARTIC.checkMCUinstructionProgress(&progress); // Check the instruction progress
      Serial.println(F("Test 7 Step 1: ARTIC R2 instruction progress:"));
      myARTIC.printInstructionProgress(progress); // Pretty-print the progress to Serial

      if (instructionComplete)
      {
        Serial.println(F("Test 7: Transmission is complete!"));

        if (digitalRead(INT1_Pin) != HIGH)
        {
          Serial.println("Test 7 Step 4: INT1_Pin is not high!");
          fail();
        }

        // Clear INT1
        if (myARTIC.clearInterrupts(1) == false)
        {
          Serial.println(F("Test 7 Step 5: clearInterrupts failed!"));
          fail();
        }

        loop_step = tests_complete; // Move on
      }

      if (millis() > (testStart + testTimeout))
      {
        Serial.println(F("Test 7 Step 6: timeout!"));
        fail();
        loop_step = tests_complete; // We're done
      }
    }
    break;

    // ************************************************************************************************
    // Tests complete!
    case tests_complete:
    {
      Serial.println();
      Serial.println(F("*** Tests complete ***"));
      Serial.println();
      Serial.println();
      Serial.println();

      pinsOff();
      SPI.end(); //Power down SPI
      digitalWrite(CS_Pin, LOW); // Stop CS providing parasitic power
      digitalWrite(11, LOW); // Stop COPI providing parasitic power
      digitalWrite(13, LOW); // Stop SCK providing parasitic power
    
      if (testFailed == false) // If all tests passed, turn the green PASS LED on
      {
        digitalWrite(PASS, HIGH); // Turn on the PASS LED
      }
      else
      {
        Serial.println(F("!!! At least one test failed... Please check the test results! !!!"));
        Serial.println();
        Serial.println();
        Serial.println();
        digitalWrite(FAIL, HIGH); // Turn on the FAIL LED
      }

      loop_step = wait_for_GO; // Start over
    }
    break;

  }
}

void pinsOff(void)
{
  digitalWrite(IOTA_PWR_EN_Pin, LOW);

  digitalWrite(FAIL, LOW);
  digitalWrite(PASS, LOW);
  
  digitalWrite(GAIN8_Pin, LOW);
  digitalWrite(BOOT_Pin, LOW);
  digitalWrite(RESET_Pin, LOW); // Hold ARTIC in reset  
}

void fail() // Stop the test
{
  Serial.println();
  Serial.println(F("!!! TEST FAILED! !!!"));
  Serial.println();
  testFailed = true;
#ifndef noFail
  loop_step = tests_complete;
#endif
}

bool GOclick(void) // Check if GO has been pressed. If it has, check for debounce on release.
{
  int highCount = 0;

  if (digitalRead(GO) == HIGH)
  {
    return (false);
  }
  else // Button has been pressed
  {
    while (highCount < 100) // Wait until GO has been high for 100 millis
    {
      if (digitalRead(GO) == LOW)
      {
        highCount = 0;
      }
      else
      {
        highCount++;
      }
      delay(1);
    }
    return (true);
  }
}
