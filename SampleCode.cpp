
#include "SampleCode.h"


// motor test ----------------------------
void motorTest (void) {
  while (true) {
    for (int i=0; i<9; i++) {
      for (int j=0; j<4; j++) {
        if ( i<4 )
          robot.motorSet( j, i==j ? 100 : 0 );
        else
          robot.motorSet( j, (i-4)==j ? -100 : 0 );
      }
      do {
        robot.buttons();
        delay(10);
      } while ( ! robot.buttonPressed[2] );
    }
  }
}



void Test (void) {
  /*
    Serial.println("start: ");
    unsigned int rread, gread, bread;
    timeTaken = millis();
    for (uint16_t k=0; k<100; k++) {
     //robot.motorControl(k/10, 100, 0);
     //robot.compoundEyes();
     //robot.Compass = robot.rawCompass();
     for (int i=0; i<4; i++) {
       //robot.rawColor(i, rread, gread, bread);
       //robot.GroundColor[i] = robot.colorSenseRead(i);
       robot.Xsonic[i] = robot.rawUltrasonic(i);
     }
    }
    timeTaken = millis() - timeTaken;
    robot.motorControl(0, 0, 0);
    Serial.print("Time taken: ");
    Serial.println(timeTaken);
    delay (1000);
  */
}

/*
  uint16_t ir[50];
  for (uint8_t i = 0; i<50; i++) {
    ir[i] = robot.rawCompoundEye(1);
    delayMicroseconds(10);
  }
  for (uint8_t i = 0; i<50; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
    if (i % 10 == 0)
      Serial.println(" ");
  }
*/

/*
void Robot::strategy() {
  int16_t angularDirection = 0;
  int16_t x, y;
  static int16_t stuckTime;
  static bool stuckFlag;
  
  if ( !autoScanEnabled ) {
    autoScanEnabled = true;
    compassRead();
    compoundEyes();
    for (uint8_t i=0; i<4; i++ ) {
      colorSenseRead(i);
      xsonicRead(i);
    }
  }
  
  if ( Eye[MaxEye] > EYEBOUNDARY )
    moveSmart(eyeAngle, 80);
  else
    moveSmart(0, 0);          // Stop
  
  return;
  
  x = (Xsonic[left] - Xsonic[right])/2;
  y = Xsonic[back] - 10;
  
  if ( Eye[MaxEye] > EYEBOUNDARY ) {
    if ( outBound[left] )
      moveSmart(90, 80);
    else if ( outBound[right] )
      moveSmart(270, 80);
    else if ( onBound[left] ) {
      switch (MaxEye) {
        case 1:
        case 12:
          moveSmart(0, 100);          // move front
        break;
        case 11:
          moveSmart(0, 0);          // Stop
        break;
        case 10:
        case 9:
        case 8:
          moveSmart(180, 100);          // move back
        break;
        default:
          moveSmart(eyeAngle, 80);
      }
    }
    else if ( onBound[right] ) {
      switch (MaxEye) {
        case 1:
        case 2:
          moveSmart(0, 100);          // move front
        break;
        case 3:
          moveSmart(0, 0);          // Stop
        break;
        case 4:
        case 5:
        case 6:
          moveSmart(180, 100);          // move back
        break;
        default:
          moveSmart(eyeAngle, 80);
      }
    }
    else if ( outBound[front] && outBound[front] )
      moveSmart(0, 0);          // Stop
    else {
      moveSmart(eyeAngle, 80);
    }
  }
  else {
    if ( stuckFlag ) {
      if ( Xsonic[back] < 35 && millis() - stuckTime < 500 )
        moveSmart(0, 60);         // move front
      else 
        stuckFlag = false;
    }
    if ( Xsonic[left]+Xsonic[right] < 45 && Xsonic[back] < 25 ) {
      stuckFlag = true;
      stuckTime = millis();
    }
    
    if ( y > 8 ) {
      if ( x > 8 ) {
        if ( y > x )
          motorDir = 7;
        else 
          motorDir = 8;
      }
      else if ( x < -8 ) {
        if ( y > -x )
          motorDir = 5;
        else 
          motorDir = 4;
      }
      else 
      moveSmart(180, 80);       // move back
    }
    else 
      
    
    if ( x > 5 )
      moveSmart(270, 80);       // move left
    else if ( x < -5 )
      moveSmart(90, 80);        // move right
    else if ( y > 5 )
      moveSmart(180, 80);       // move back
    else if ( y < -5 )
      moveSmart(0, 60);         // move front
    else {
      moveSmart(0, 0);          // Stop
      for (uint8_t i=0; i<4; i++ )
        outBound[i] = false;
    }
  }
}
*/




