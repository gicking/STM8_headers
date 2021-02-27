import serial
import logging
import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus_rtu as modbus_rtu
import time

# 16-bit client commands via "write registers". All commands have highest bit set (bit15).
# After command has completed, bit15 is cleared by the client.
CMD_SET_LED             = 0x8001            # set LED
CMD_REG_RANDOMIZE       = 0x8002            # fill registers with random numbers
CMD_ID_RANDOMIZE	    = 0x8003            # randomize Modbus IDs

# error codes from client (in reg[1])
ERROR_ILLEGAL_COMMAND   = 1					# unknown command code
ERROR_PARAMETER_NUMBER  = 2                 # wrong number of parameters for command

# number of Modbus registers to read or write
NUM_REGS                = 16



def set_led(id, state):
    try:
        logger.info(master.execute(slave=id, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=0, output_value=[CMD_SET_LED, state]))
    except modbus_tk.modbus.ModbusError as e:
        logger.error("%s- Code=%d" % (e, e.get_exception_code()))
    except modbus_tk.modbus_rtu.ModbusInvalidResponseError as e:
        logger.error("%s- Code=ModbusInvalidResponseError" % (e))
        master.close()
    except:
        logger.debug("other")



def scan_modbus_id():

    # broadcast to all clients to randomize registers to ensure checksum error in case of ID collision
    logger.info(master.execute(slave=0, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=0, output_value=[CMD_REG_RANDOMIZE]))

    # set short timeout to reduce time for scan
    old_timeout = master.get_timeout()
    master.set_timeout(0.02)
    time.sleep(0.1)

    # scan over valid ID range (1..247) and check client response
    valid = []
    conflict = []
    unused = []
    #for id in range(1,4):
    for id in range(1,247+1):
        try:
            master.execute(slave=id, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=5)
            time.sleep(0.01)
            master.execute(slave=id, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=5)
            time.sleep(0.01)
            master.execute(slave=id, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=5)
            valid.append(id)
        except modbus_tk.modbus.ModbusError as e:
            #logger.error("IDx %d: %s- Code=%d" % (id, e, e.get_exception_code()))
            conflict.append(id)
        except modbus_tk.modbus_rtu.ModbusInvalidResponseError as e:
            #logger.error("IDy %d: %s- Code=ModbusInvalidResponseError" % (id, e))
            if str(e) == "Response length is invalid 0":
                unused.append(id)
            else:
                conflict.append(id)
        except:
            pass
            logger.debug("IDz %d: other error" % (id))
        time.sleep(0.01)

    # restore original timeout
    master.set_timeout(old_timeout)
    time.sleep(0.1)

    # return list of valid and conflicting IDs
    return valid, conflict



def randomize_registers():
    try:
        logger.info(master.execute(slave=0, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=0, output_value=[CMD_REG_RANDOMIZE]))
    except modbus_tk.modbus.ModbusError as e:
        logger.error("%s- Code=%d" % (e, e.get_exception_code()))
    except modbus_tk.modbus_rtu.ModbusInvalidResponseError as e:
        logger.error("%s- Code=ModbusInvalidResponseError" % (e))
        master.close()
    except:
        logger.debug("other")
    time.sleep(0.01)



def randomize_id(id):
    try:
        logger.info(master.execute(slave=0, function_code=cst.WRITE_MULTIPLE_REGISTERS, starting_address=0, output_value=[CMD_ID_RANDOMIZE, id]))
    except modbus_tk.modbus.ModbusError as e:
        logger.error("%s- Code=%d" % (e, e.get_exception_code()))
    except modbus_tk.modbus_rtu.ModbusInvalidResponseError as e:
        logger.error("%s- Code=ModbusInvalidResponseError" % (e))
        master.close()
    except:
        logger.debug("other")
    time.sleep(0.01)



def read_registers(id, num=2):
    reg = []
    try:
        reg = master.execute(slave=id, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=num)
        #logger.info(master.execute(slave=id, function_code=cst.READ_INPUT_REGISTERS, starting_address=0, quantity_of_x=num))
    except modbus_tk.modbus.ModbusError as e:
        logger.error("%s- Code=%d" % (e, e.get_exception_code()))
    except modbus_tk.modbus_rtu.ModbusInvalidResponseError as e:
        logger.error("%s- Code=ModbusInvalidResponseError" % (e))
        master.close()
    except:
        logger.debug("other")
    print(reg)



if __name__ == "__main__":

    # create logging instance
    logger = modbus_tk.utils.create_logger("console")
    logger.setLevel(level=logging.ERROR)

    # Connect to the slave
    try:
        master = modbus_rtu.RtuMaster(
            serial.Serial(port="/dev/ttyUSB1", baudrate=19200, bytesize=8, parity='N', stopbits=1, xonxoff=0))
        master.set_timeout(0.2)
        master.set_verbose(True)
        logger.info("connected")
        delay = 0.005
    except:
        logger.error("connection failed")

    delay = 0.5  # [s]
    while True:

        # check for clients in network
        print("start scan ... ", end="", flush=True);
        time_start = time.time()
        valid, conflict = scan_modbus_id()
        time_end = time.time()
        print("done (" + str(round(time_end-time_start, 1)) + "s)", flush=True)

        # print valid and conflicting IDs
        print("valid IDs: ", end = " ")
        for id in valid:
            print(hex(id), end = " ")
        print()
        print("conflicting IDs: ", end = " ")
        for id in conflict:
            print(hex(id), end = " ")
        print("\n", flush=True)


        # for valid IDs, toggle LED
        for id in valid:
            set_led(id=id, state=1)
            time.sleep(delay)
            set_led(id=id, state=0)
            time.sleep(delay)


        # for conflicting IDs request new IDs
        for id in conflict:
            randomize_id(id)

        time.sleep(1)