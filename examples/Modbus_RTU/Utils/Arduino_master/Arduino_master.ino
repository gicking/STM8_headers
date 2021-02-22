/**
    Modbus master example 1:
    The purpose of this example is to query an array of data
    from an external Modbus slave device.
    The link media can be USB or RS232.

    Recommended Modbus slave:
    diagslave http://www.modbusdriver.com/diagslave.html

    In a Linux box, run
    "./diagslave /dev/ttyUSB0 -b 19200 -d 8 -s 1 -p none -m rtu -a 1"
  	This is:
  		serial port /dev/ttyUSB0 at 19200 baud 8N1
 		RTU mode and address @1
*/

#include <ModbusRtu.h>




/**
    Modbus object declaration
    u8id : node id = 0 for master, = 1..247 for slave
    port : serial port
    u8txenpin : 0 for RS-232 and USB-FTDI
                 or any pin number >1 for RS-485
*/
Modbus master(0, Serial1, 2);


/**
   This is an structe which contains a query to an slave device
*/
modbus_t telegram;


void write_registers(uint8_t id, uint8_t addr, uint8_t num, uint16_t *buf)
{
  telegram.u8id = id;                 // slave address (0=broadcast)
  telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;  // function code
  telegram.u16RegAdd = addr;          // start address in slave
  telegram.u16CoilsNo = num;          // number of elements (coils or registers) to read
  telegram.au16reg = buf;             // pointer to a memory array in the Arduino
  master.query( telegram );           // send query (only once)

  // for unicast command receive response
  if (id != 0)
  { 
    while (master.getState() != COM_IDLE)
      master.poll(); // check incoming messages

    if (master.getTimeOutState() != 0)
      Serial.println("write_registers(): response timeout");
  }
  
  // for broadcast wait until bus is idle (no client response)
  else
  {
    while (master.getState() != COM_IDLE)
      ;
  }
  
}


void read_registers(uint8_t id, uint8_t addr, uint8_t num, uint16_t *buf)
{
  telegram.u8id = id;                 // slave address
  telegram.u8fct = MB_FC_READ_INPUT_REGISTER;  // function code
  telegram.u16RegAdd = addr;          // start address in slave
  telegram.u16CoilsNo = num;          // number of elements (coils or registers) to read
  telegram.au16reg = buf;             // pointer to a memory array in the Arduino
  master.query( telegram );           // send query (only once)

  while (master.getState() != COM_IDLE)
    master.poll(); // check incoming messages

  if (master.getTimeOutState() != 0)
    Serial.println("read_register(): response timeout");
}


void set_LED(uint8_t id, uint8_t state)
{
  uint16_t buf[] = {state};
  write_registers(id, 0, 1, buf);
}


uint16_t read_time(uint8_t id)
{
  uint16_t buf[8];
  read_registers(id, 0, 1, buf);
  return buf[0];
}



void setup()
{
  Serial.begin( 115200 ); // debug console

  Serial1.begin( 115200 ); // baud-rate at 19200
  master.start();
  master.setTimeOut( 200 ); // if there is no answer in 2000 ms, roll over
}


void loop() {

  int pause = 500;

  set_LED(1, 0); delay(pause);
  set_LED(2, 0); delay(pause);
  set_LED(1, 1); delay(pause);
  set_LED(2, 1); delay(pause);
  
  Serial.println((int) read_time(1));
  delay(pause);
}
