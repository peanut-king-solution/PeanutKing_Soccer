#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup(void) {
  robot.init();
}

void loop() {
  static uint32_t dataFetchTimer = 0;
  robot.bluetoothRemote();
  
  if (millis() - dataFetchTimer > 50) {
    robot.dataFetch();
    dataFetchTimer = millis();
  }
}

void bluetoothRemote(void) {
  static btData_t btDataHeader = Idle;
  static uint32_t btSendTimer = 0;
  static uint8_t btState = 0, len = 0;
  static int btAngle = 0;
  static String deg = "", dis = "", buttonVal = "";
  static float speed = 1.0;

  static char idx = 0;
  char i = 0;
  
  if (Serial1.available()) {
    btRxBuffer[idx] = Serial1.read();
    // Serial.print(btRxBuffer[idx]);
    if (btRxBuffer[idx] == 'Z') {
      btDataHeader = EndOfData;
    }
    idx++;
  }

  if (btDataHeader == EndOfData) {
    for () {

    }
    switch (btDataHeader) {
      case Idle:
      case EndOfData:
      case DemoMode:
        switch (btRxBuffer[idx]) {
          case 'A':
            btDataHeader = Joystick;
            break;
          case 'B':
            btDataHeader = PadButton;
            break;
          case 'C':
            btDataHeader = ButtonDef;
            break;
          case 'D':
            //btDataHeader = Attributes;
            break;
          case 'Z':
            btDataHeader = EndOfData;
            break;
          case 'Y':
            btDataHeader = DemoMode;
            break;
        }
        btState = 1;
        break;
      case Joystick:
        switch(btState) {
          case 1:
            if (v != 'D')
              deg += v;
            else {
              int temp = deg.toInt();
              if (temp<360)
                btDegree = temp;
                
              deg = "";
              btState++;
              // Serial.print(btDegree); Serial.print(' ');
            }
          break;
          case 2:
            if (v != '.')
              dis += v;
            else {
              int temp = dis.toInt();
              if (temp<=100)
                btDistance = temp;
                
              //Serial.print(btDistance); Serial.println(' ');
              dis = "";
              btState=0;
              btDataHeader = Idle;
            }
          break;
        }
        break;
      case PadButton:
        if (len==0) {
          btButtonIndex = v-'0';
          len ++;
        }
        else if (len==1) {
          btGestureCode = v-'0';
          len = 0;
          btDataHeader = Idle;
          //Serial.print("buttun pressed ");
          //Serial.print(btButtonIndex); Serial.print(btGestureCode); Serial.println(' ');
          //List<String> functionList = ['Accel', 'Back', 'Chase', 'Auto', 'L-Trun', 'R-Trun', 'Front', 'Left', 'Right', 'Back'];
        }
        break;
      case ButtonDef:
        if (len<4) {
          //Serial.print("buttun code ");
          //Serial.print(v);
          //Serial.println(' ');
          //String code = v;
          btButtonFunction[len] = v-'0';//code.toInt();
          len ++;
        }
        else {
          len = 0;
          //buttonVal = "";
          //btState++;
          //btState = 0;
          btDataHeader = Idle;
        }
        break;
      case Attributes:
        if (len<5) {
          btAttributes[len] = v-'0';//code.toInt();
          len ++;
        }
        else {
          EYEBOUNDARY = 10 + (10-btAttributes[0]) * 20;
          len = 0;
          btDataHeader = Idle;
        }
        break;
    }














    btDataHeader = Idle;
    //Serial.print("buttun pressed ");
    //Serial.print(btButtonIndex); Serial.print(btGestureCode); Serial.println(' ');

    if ( btGestureCode==0 || btGestureCode==4 ) {
      switch (btButtonFunction[btButtonIndex]) {
        case 0:   // Accel
          speed = 2.0;
          break;
        case 1:   // Back
          Back(btDegree, btDistance, btRotate);
          break;
        case 2:   // Chase
          Chase(btDegree, btDistance, btRotate);
          break;
        case 3:   // Auto
          if ( eye[maxEye] > 30 ) {
            Chase(btDegree, btDistance, btRotate);
          }
          else
          {
            btDistance = 0;
            btDegree = 0;
          }
          
          //Back(btDegree, btDistance, btRotate);
          break;
        case 4:
          btAngle = compass;
          btRotate = -40;
          break;
        case 5:
          btAngle = compass;
          btRotate = 40;
          break;
        case 6:   // Front
          break;
      }
    }
    else { // if ( btGestureCode==3 || btGestureCode==6 )
      if (btRotate ==-40 || btRotate ==40) {
          btAngle = compass;
      }
      speed = 1.0;
      btRotate = 0;
    }
    // Execute
    
      Serial.print(btDegree); Serial.print(' ');
      Serial.print(btDistance); Serial.print(' ');
      Serial.print(btRotate); Serial.println(' ');
    
  }

  // if ( btRotate==0 ) {
  //   moveSmart(btDegree, btDistance*speed*0.7, btAngle);
  //   motorControl(btDegree, btDistance, btRotate);
  // }
  // else
  //   motorControl(0, 0, btRotate);

  /*
  else if (btDataHeader == DemoMode) {
    if ( eye[maxEye] > EYEBOUNDARY ) {
      Chase(btDegree, btDistance, btRotate);
    }
    //Back(btDegree, btDistance, btRotate);
    moveSmart(btDegree, btDistance*speed, btAngle);
  }*/

  // Send Data
  if (millis() - btSendTimer > 100) {
    if (Serial1.availableForWrite() > 50) {
      btTxBuffer[0] = 'C';
      btTxBuffer[1] = compass & 0xff;
      btTxBuffer[2] = compass >> 8;
      btTxBuffer[3] = 'U';
      btTxBuffer[4] = ultrasonic[0] > 255 ? 255 : ultrasonic[0];
      btTxBuffer[8] = 'E';
      btTxBuffer[9] = maxEye;
      btTxBuffer[11] = eye[maxEye] & 0xff;
      btTxBuffer[12] = eye[maxEye] >> 8;
      btTxBuffer[13] = 'Z';
      Serial1.write(btTxBuffer, 14);
    }
    btSendTimer = millis();
  }
}
