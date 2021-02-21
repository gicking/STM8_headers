#!/usr/bin/env python

import sys
import serial

# add logging capability
import logging

import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
import time

logger = modbus_tk.utils.create_logger("console")

if __name__ == "__main__":

    for i in range(1):
        #while True:

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port="/dev/ttyUSB0", baudrate=115200, bytesize=8, parity='N', stopbits=1, xonxoff=0))
            master.set_timeout(2.0)
            master.set_verbose(True)
            logger.info("connected")

            #logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 100, 3))

            delay=0.005  # [s]
            while True:
                logger.info(master.execute(1, cst.WRITE_SINGLE_COIL, 0, output_value=0))
                time.sleep(delay)
                logger.info(master.execute(2, cst.WRITE_SINGLE_COIL, 0, output_value=0))
                time.sleep(delay)
                logger.info(master.execute(1, cst.WRITE_SINGLE_COIL, 0, output_value=1))
                time.sleep(delay)
                logger.info(master.execute(2, cst.WRITE_SINGLE_COIL, 0, output_value=1))
                time.sleep(delay)

            while True:
                logger.info(master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 0, output_value=[1,3,5,7]))
                logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 4))
                logger.info(master.execute(1, cst.READ_COILS, starting_address=0, quantity_of_x=3))
                logger.info(master.execute(1, cst.READ_DISCRETE_INPUTS, 0, 8))
                logger.info(master.execute(1, cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=4))
                logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 4))
                logger.info(master.execute(1, cst.WRITE_SINGLE_COIL, 0, output_value=0))
                logger.info(master.execute(1, cst.WRITE_SINGLE_REGISTER, 0, output_value=54))
                logger.info(master.execute(1, cst.WRITE_MULTIPLE_COILS, 0, output_value=[1, 1, 0, 1, 1, 0, 1, 1]))

        except modbus_tk.modbus.ModbusError, e:
            logger.error("%s- Code=%d" % (e, e.get_exception_code()))

        time.sleep(1.0)