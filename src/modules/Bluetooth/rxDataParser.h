#ifndef RX_DATA_PARSER_H
#define RX_DATA_PARSER_H

// PILA remote control command codes
enum PILA_RX_Cmd
{
  CMD_FORWARD  = 'F',
  CMD_BACKWARD = 'B',
  CMD_LEFT     = 'L',
  CMD_RIGHT    = 'R',
  CMD_CIRCLE   = 'C',
  CMD_CROSS    = 'X',
  CMD_TRIANGLE = 'T',
  CMD_SQUARE   = 'S',
  CMD_START    = 'A',
  CMD_LIGHT    = 'Z',
  CMD_PAUSE    = 'P'
};

class rxDataParser
{
private:
  /* data */
public:
  // Constructor
  rxDataParser(/* args */);
  // Destructor
  ~rxDataParser();
};

#endif // RX_DATA_PARSER_H