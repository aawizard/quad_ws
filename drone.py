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
HOST = '192.168.18.103'  # Server IP
PORT = 65431
# def run_curses(external_function):
#     result=1

#     try:
#         # get the curses screen window
#         screen = curses.initscr()

#         # turn off input echoing
#         curses.noecho()

#         # respond to keys immediately (don't wait for enter)
#         curses.cbreak()

#         # non-blocking
#         screen.timeout(0)

#         # map arrow keys to special values
#         screen.keypad(True)

#         screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm and arrow keys to control", curses.A_BOLD)
        
#         result = external_function(screen)
#         # result = 0

#     finally:
#         # shut down cleanly
#         curses.nocbreak(); screen.keypad(0); curses.echo()
#         curses.endwin()
#         if result==1:
#             print("An error occurred... probably the serial port is not available ;)")
#         else:
#             print("Th..th..th.thhhhat's all folks!")

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

            # screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            # screen.clrtoeol()
            # screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            # screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            # screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            # screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            # screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))


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
                    # print(drone_cmds_)
                    # drone_cmds = [1500, 1500, 900, 1500, 1800, 1000]
                    for i, cmd in enumerate(CMDS_ORDER):
                        CMDS[cmd] = drone_cmds[i]
                    print(CMDS['aux1'], drone_cmds[4], "\n")
                    print(drone_cmds[-1])
                    # char = screen.getch() # get keypress
                    # curses.flushinp() # flushes buffer
                    

                    #
                    # Key input processing
                    #

                    #
                    # KEYS (NO DELAYS)
                    #
                    # if char == ord('q') or char == ord('Q'):
                    #     break

                    # elif char == ord('d') or char == ord('D'):
                    #     cursor_msg = 'Sending Disarm command...'
                    #     CMDS['aux1'] = 1000

                    # elif char == ord('r') or char == ord('R'):
                    #     screen.addstr(3, 0, 'Sending Reboot command...')
                    #     screen.clrtoeol()
                    #     board.reboot()
                    #     time.sleep(0.5)
                    #     break

                    # elif char == ord('a') or char == ord('A'):
                    #     cursor_msg = 'Sending Arm command...'
                    #     CMDS['aux1'] = 1800

                    # #
                    # # The code below is expecting the drone to have the
                    # # modes set accordingly since everything is hardcoded.
                    # #
                    # elif char == ord('m') or char == ord('M'):
                    #     if CMDS['aux2'] <= 1300:
                    #         cursor_msg = 'Horizon Mode...'
                    #         CMDS['aux2'] = 1500
                    #     elif 1700 > CMDS['aux2'] > 1300:
                    #         cursor_msg = 'Flip Mode...'
                    #         CMDS['aux2'] = 2000
                    #     elif CMDS['aux2'] >= 1700:
                    #         cursor_msg = 'Angle Mode...'
                    #         CMDS['aux2'] = 1000

                    # elif char == ord('w') or char == ord('W'):
                    #     CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                    #     cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])

                    # elif char == ord('e') or char == ord('E'):
                    #     CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                    #     cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])

                    # elif char == curses.KEY_RIGHT:
                    #     CMDS['roll'] = CMDS['roll'] + 10 if CMDS['roll'] + 10 <= 2000 else CMDS['roll']
                    #     cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                    # elif char == curses.KEY_LEFT:
                    #     CMDS['roll'] = CMDS['roll'] - 10 if CMDS['roll'] - 10 >= 1000 else CMDS['roll']
                    #     cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                    # elif char == curses.KEY_UP:
                    #     CMDS['pitch'] = CMDS['pitch'] + 10 if CMDS['pitch'] + 10 <= 2000 else CMDS['pitch']
                    #     cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                    # elif char == curses.KEY_DOWN:
                    #     CMDS['pitch'] = CMDS['pitch'] - 10 if CMDS['pitch'] - 10 >= 1000 else CMDS['pitch']
                    #     cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])

                    #
                    # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                    #
                    if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        # Send the RC channel values to the FC
                        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)

                    #
                    # SLOW MSG processing (user GUI)
                    #
                    # if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    #     last_slow_msg_time = time.time()

                    #     next_msg = next(slow_msgs) # circular list

                    #     # Read info from the FC
                    #     if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                    #         dataHandler = board.receive_msg()
                    #         board.process_recv_data(dataHandler)
                            
                    #     if next_msg == 'MSP_ANALOG':
                    #         voltage = board.ANALOG['voltage']
                    #         voltage_msg = ""
                    #         if min_voltage < voltage <= warn_voltage:
                    #             voltage_msg = "LOW BATT WARNING"
                    #         elif voltage <= min_voltage:
                    #             voltage_msg = "ULTRA LOW BATT!!!"
                    #         elif voltage >= max_voltage:
                    #             voltage_msg = "VOLTAGE TOO HIGH"

                    #         screen.addstr(8, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                    #         screen.clrtoeol()
                    #         screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                    #         screen.clrtoeol()
                    #         screen.addstr(20,0, "Battery Amperage: {:2.2f}A".format(board.ANALOG['amperage']))
                    #         screen.clrtoeol()
                    #         screen.addstr(22,0, "Battery Power: {:2.2f}W".format(board.ANALOG['voltage'] * board.ANALOG['amperage']))
                    #         screen.clrtoeol()

                    #     elif next_msg == 'MSP_STATUS_EX':
                    #         # ARMED = board.bit_check(board.CONFIG['mode'],0)
                    #         #it seems this is the 'better' way to figure out if the darn thing is armed or not. (board.bit_check(board.CONFIG['mode'],0)) wasn't quite working
                    #         flight_mode = board.process_mode(board.CONFIG['mode'])
                    #         if 'ARM' in flight_mode:
                    #             ARMED = True
                    #         else:
                    #             ARMED = False
                    #         screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                    #         screen.clrtoeol()

                    #         screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                    #         screen.clrtoeol()

                    #         screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                    #         screen.clrtoeol()
                    #         screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                    #         screen.clrtoeol()

                    #         screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                    #         screen.clrtoeol()

                    #         screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                    #         screen.clrtoeol()


                    #     elif next_msg == 'MSP_MOTOR':
                    #         screen.addstr(9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                    #         screen.clrtoeol()

                    #     elif next_msg == 'MSP_RC':
                    #         screen.addstr(10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                    #         screen.clrtoeol()

                    #     # screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                    #     #                                                                             1/(sum(average_cycle)/len(average_cycle))))
                    #     screen.clrtoeol()

                    #     screen.addstr(3, 0, f"{drone_cmds}")
                    #     screen.clrtoeol()
                        

                    end_time = time.time()
                    last_cycleTime = end_time-start_time
                    if (end_time-start_time)<CTRL_LOOP_TIME:
                        time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                        
                    average_cycle.append(end_time-start_time)
                    average_cycle.popleft()

    finally:
        print("done")
        # board.close()
        # screen.addstr(5, 0, "Disconneced from the FC!")
        # screen.clrtoeol()

if __name__ == "__main__":
    server_controller()


# import socket
# import json
# import time
# import curses
# from collections import deque

# # Configuration constants
# SERVER_HOST = '192.168.18.103'  # Server IP (localhost for testing on local machine)
# SERVER_PORT = 65437          # The same port number as the server
# CTRL_LOOP_TIME = 1/100      # 100 Hz control loop
# NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

# def run_curses(external_function):
#     result = 1

#     try:
#         # Get the curses screen window
#         screen = curses.initscr()

#         # Turn off input echoing
#         curses.noecho()
#         # Respond to keys immediately (don't wait for enter)
#         curses.cbreak()
#         # Non-blocking
#         screen.timeout(0)
#         # Map arrow keys to special values
#         screen.keypad(True)

#         result = external_function(screen)

#     finally:
#         # Shut down cleanly
#         curses.nocbreak(); screen.keypad(0); curses.echo()
#         curses.endwin()
#         if result == 1:
#             print("An error occurred... probably the server is not available ;)")
#         else:
#             print("Th..th..th.thhhhat's all folks!")

# def server_controller(screen):
#     # Create a socket object
#     conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#     try:
#         # Connect to the server
#         conn.connect((SERVER_HOST, SERVER_PORT))
#         screen.addstr(0, 0, "Connected to the server!")
#         screen.clrtoeol()

#         # Average cycle time
#         average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

#         # Receive data loop
#         buffer = ""
#         while True:
#             start_time = time.time()

#             # Receive data (make sure the buffer size is sufficient)
#             data = conn.recv(1024).decode('utf-8')  # Adjust buffer size as needed
#             if not data:
#                 break  # Connection closed

#             buffer += data
            
#             # Split the buffer by newlines and process each line
#             lines = buffer.split('\n')
            
#             # Keep the last incomplete line in the buffer
#             buffer = lines.pop() if buffer.endswith('\n') else lines[-1]
            
#             for line in lines:
#                 if line.strip():  # Ignore empty lines
#                     try:
#                         # Parse each line as JSON
#                         drone_cmds = json.loads(line)  
#                         print(drone_cmds)
#                         screen.addstr(1, 0, f"Received commands: {drone_cmds}")
#                         screen.clrtoeol()
#                     except json.JSONDecodeError as e:
#                         screen.addstr(1, 0, f"Failed to decode JSON: {e}")
#                         screen.clrtoeol()

#             # Control loop timing
#             end_time = time.time()
#             cycle_time = end_time - start_time
#             average_cycle.append(cycle_time)
#             average_cycle.popleft()

#             if (end_time - start_time) < CTRL_LOOP_TIME:
#                 time.sleep(CTRL_LOOP_TIME - (end_time - start_time))

#     finally:
#         conn.close()
#         screen.addstr(2, 0, "Disconnected from the server!")
#         screen.clrtoeol()

# if __name__ == "__main__":
#     run_curses(server_controller)
