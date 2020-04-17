
typedef enum {
  Idle,
  SensorTypeReceived,
  CMDReceived,
  DataReceived,
  
} protocolStatues;
/*
'C' - Compass 
'U' - Ultrasonic
'I' - IR sensor
'C' - Color sensor
- Motor
- LED
- LCD Screen
*/
typedef struct {
  uint8_t type;
  uint9_t length;
  uint8_t data;
} sensor_t;

readOneByte();
sendOneByte();
