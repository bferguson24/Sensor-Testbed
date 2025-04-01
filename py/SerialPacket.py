from enum import Enum
import struct


class Command(Enum):
    COMMAND_IDLE = 0x00
    COMMAND_START = 0x01
    COMMAND_STOP = 0x02
    COMMAND_HOME = 0x03 
    COMMAND_MOVE_WAYPOINT = 0x04
    COMMAND_MOVE_MANUAL = 0x05



def send_packet(serial_port, command, x, y, pitch, vibe):

    if not serial_port.is_open:
        serial_port.open()  
        
    sync_word = b'\xDA\xBA\xD0\x00'  # 4-byte sync word
    message_length = 1 + 4 + 4 + 4 + 4  # Command (1 byte) + x (4 bytes) + y (4 bytes) + z (4 bytes) + vibe (4 bytes)
    packet = struct.pack('<4sBBffff', sync_word, message_length, command.value, x, y, pitch, vibe)

    hex_packet = ' '.join(f'{byte:02X}' for byte in packet)
    # print(f'Raw packet in hex: {hex_packet}')
    
    # Send the packet and print confirmation
    serial_port.write(packet)
    print(f"Sent packet: Command={command}, X={x}, Y={y}, Pitch={pitch}, Vibe = {vibe}")