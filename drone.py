#derived from: https://github.com/thecognifly/YAMSPy/blob/master/Examples/simpleUI.py

import time
import curses
from collections import deque
from itertools import cycle
import socket
from yamspy import MSPy
import json
print('foo')

# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

SERIAL_PORT = "/dev/serial0"
HOST = '192.168.18.101'  # Server IP
PORT = 65432

def server_controller():

    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 900,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes AUX1, AUX2...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    # "print" doesn't work with curses, use addstr instead
    try:
        # screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            # screen.addstr(15, 0, "Connecting to the FC... connected!")
            # screen.clrtoeol()
            # screen.move(1,0)

            

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount


            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            buffer=""
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((HOST, PORT))

                # screen.addstr(3, 0, 'Connected to server for commands.')
            
                average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)
                last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
                while True:
                    start_time = time.time()
                    data = s.recv(1024).decode('utf-8')  
                    # print("here")
                    # print(data)
                    if not data:
                        break
                    
                    buffer += data
            
                    # Split the buffer by newlines and process each line
                    lines = buffer.split('\n')
                    
                    # Keep the last incomplete line in the buffer
                    buffer = lines.pop() if buffer.endswith('\n') else lines[-1]
                    
                    for line in lines:
                        if line.strip():  # Ignore empty lines
                            try:
                                # Parse each line as JSON
                                drone_cmds = json.loads(line)  
                                print(drone_cmds, "no error")
                                # screen.addstr(1, 0, f"Received commands: {drone_cmds}")
                                # screen.clrtoeol()
                            except json.JSONDecodeError as e:
                                print("error")
                                # screen.addstr(1, 0, f"Failed to decode JSON: {e}")
                                # screen.clrtoeol()
                            
                    # Decode and parse the list of commands
                    # drone_cmds_ = json.loads(data.decode('utf-8'))
                    for i, cmd in enumerate(CMDS_ORDER):
                        CMDS[cmd] = drone_cmds[i]
                    print(CMDS['aux1'], drone_cmds[4], "\n")
                    print(drone_cmds[-1])
                    

                    #
                    if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        # Send the RC channel values to the FC
                        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                        

                    end_time = time.time()
                    last_cycleTime = end_time-start_time
                    if (end_time-start_time)<CTRL_LOOP_TIME:
                        time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                        
                    average_cycle.append(end_time-start_time)
                    average_cycle.popleft()

    finally:
        print("done")

if __name__ == "__main__":
    server_controller()
