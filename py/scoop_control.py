import serial
import time
from SerialPacket import *



#Read Waypoints
def read_waypoint(filename):
    coordinates = []
    with open(filename, 'r') as file:
        for line in file:
            x,y = map(float, line.strip().split(','))
            coordinates.append((x,y))
    return coordinates 



class Scoop:
    
    def __init__(self):
        self.ser = serial.Serial(port = "COM12", baudrate = 115200)
        self.command = Command.COMMAND_STOP
        self.x = 0.0
        self.y = 0.0
        self.pitch = 0.0
        self.vibe = 0.0

    def update_values(self, pitch = 0.0, vibe = 0.0):
        self.pitch = pitch
        self.vibe = vibe

    def idle(self):
        self.command = Command.COMMAND_IDLE
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)
    
    def start(self):
        self.command = Command.COMMAND_START
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)
        
    def stop(self):
        self.command = Command.COMMAND_STOP
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)


    def home(self):
        self.command = Command.COMMAND_HOME
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)

    def move_position(self, x, y, pitch, vibe):
        self.x = x
        self.y = y
        self.pitch = pitch
        self.vibe = vibe




    def dig_sequence(self):
        self.command = Command.COMMAND_MOVE_WAYPOINT
        traj = read_waypoint('waypoints.txt')
        
        i = 0
        try:
            while (i < len(traj)):
                x,y = traj[i]
                send_packet(self.ser, command= self.command, x=x, y=y, pitch = self.pitch, vibe = self.vibe)
                time.sleep(0.01)  # Sleep for 0.01 seconds (100 Hz frequency)
                i = i + 1
            self.idle() 
            
        finally:
            self.ser.close()

    def manual_control(self):
        self.command = Command.COMMAND_MOVE_MANUAL
        send_packet(self.ser, self.command, self.x, self.y, self.pitch, self.vibe)


        

