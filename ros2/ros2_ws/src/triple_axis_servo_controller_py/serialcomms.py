"""
"""

import serial
import base64


def makeCommand(target_board_id,
                drives_enable,
                drv1_enable, drv1_reset, drv1_in1, drv1_in2,
                drv2_enable, drv2_reset, drv2_in1, drv2_in2,
                drv3_enable, drv3_reset, drv3_in1, drv3_in2):
    if target_board_id < 0 or 255 < target_board_id:
        raise ValueError('target_board_id must be in range [0,255]')
    if not isinstance(drives_enable, bool):
        raise TypeError('drive_enable must be bool')
    if not isinstance(drv1_enable, bool):
        raise TypeError('drv1_enable must be bool')
    if not isinstance(drv1_reset, bool):
        raise TypeError('drv1_reset must be bool')
    if not isinstance(drv2_enable, bool):
        raise TypeError('drv2_enable must be bool')
    if not isinstance(drv2_reset, bool):
        raise TypeError('drv2_reset must be bool')
    if not isinstance(drv3_enable, bool):
        raise TypeError('drv3_enable must be bool')
    if not isinstance(drv3_reset, bool):
        raise TypeError('drv3_reset must be bool')
    if drv1_in1 < 0 or 255 < drv1_in1:
        raise ValueError('drv1_in1 must be in range [0,255]')
    if drv1_in2 < 0 or 255 < drv1_in2:
        raise ValueError('drv1_in2 must be in range [0,255]')
    if drv2_in1 < 0 or 255 < drv2_in1:
        raise ValueError('drv2_in1 must be in range [0,255]')
    if drv2_in2 < 0 or 255 < drv2_in2:
        raise ValueError('drv2_in2 must be in range [0,255]')
    if drv3_in1 < 0 or 255 < drv3_in1:
        raise ValueError('drv3_in1 must be in range [0,255]')
    if drv3_in2 < 0 or 255 < drv3_in2:
        raise ValueError('drv3_in2 must be in range [0,255]')
    
    command = bytes([
        target_board_id,
        (drv1_enable << 6) | (drv1_reset << 5) | \
            (drv2_enable << 4) | (drv2_reset << 3) | \
            (drv3_enable << 2) | (drv3_reset << 1) | \
            drives_enable,
        drv1_in1,
        drv1_in2,
        drv2_in1,
        drv2_in2,
        drv3_in1,
        drv3_in2,
    ])
    checksum = command[0]
    for i in range(1, len(command)):
        checksum = checksum ^ command[i]
    return base64.b16encode(command + bytes([checksum])) + b'\n'

def parseResponse(input_bytes):
    if len(input_bytes) != 29:
        raise ValueError('bad input length')

    board_id = int(input_bytes[0:2], base=16)

    drv1_status = int(input_bytes[2:3], base=16)
    drv1_enabled = bool(drv1_status & 0b100)
    drv1_ready = bool(drv1_status & 0b10)
    drv1_faulted = bool(drv1_status & 0b1)

    drv2_status = int(input_bytes[3:4], base=16)
    drv2_enabled = bool(drv2_status & 0b100)
    drv2_ready = bool(drv2_status & 0b10)
    drv2_faulted = bool(drv2_status & 0b1)

    drv3_status = int(input_bytes[4:5], base=16)
    drv3_enabled = bool(drv3_status & 0b100)
    drv3_ready = bool(drv3_status & 0b10)
    drv3_faulted = bool(drv3_status & 0b1)

    pot1 = int(input_bytes[5:8], base=16)
    current1 = int(input_bytes[8:11], base=16)
    pot2 = int(input_bytes[11:14], base=16)
    current2 = int(input_bytes[14:17], base=16)
    pot3 = int(input_bytes[17:20], base=16)
    current3 = int(input_bytes[20:23], base=16)

    servo_voltage = int(input_bytes[23:26], base=16)

    checksum = board_id
    checksum = checksum ^ drv1_status
    checksum = checksum ^ drv2_status
    checksum = checksum ^ drv3_status
    checksum = checksum ^ (pot1 >> 8)
    checksum = checksum ^ (pot1 & 0xFF)
    checksum = checksum ^ (current1 >> 8)
    checksum = checksum ^ (current1 & 0xFF)
    checksum = checksum ^ (pot2 >> 8)
    checksum = checksum ^ (pot2 & 0xFF)
    checksum = checksum ^ (current2 >> 8)
    checksum = checksum ^ (current2 & 0xFF)
    checksum = checksum ^ (pot3 >> 8)
    checksum = checksum ^ (pot3 & 0xFF)
    checksum = checksum ^ (current3 >> 8)
    checksum = checksum ^ (current3 & 0xFF)
    checksum = checksum ^ (servo_voltage >> 8)
    checksum = checksum ^ (servo_voltage & 0xFF)

    if checksum != int(input_bytes[-3:-1], base=16):
        raise ValueError('bad checksum')

    return (
        board_id,
        drv1_enabled,
        drv1_ready,
        drv1_faulted,
        drv2_enabled,
        drv2_ready,
        drv2_faulted,
        drv3_enabled,
        drv3_ready,
        drv3_faulted,
        pot1 * 0.0009775171065493646,
        current1 * 0.026419381258090936 - 13.513513513513514,
        pot2 * 0.0009775171065493646,
        current2 * 0.026419381258090936 - 13.513513513513514,
        pot3 * 0.0009775171065493646,
        current3 * 0.026419381258090936 - 13.513513513513514,
        servo_voltage * 0.014951568470630052,
    )

def sendCommand(ser, command):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    _ = ser.write(command)
    return ser.readline()

def sendCommandAndPrintResponse(ser, command):
    r = sendCommand(ser, command)
    if len(r) > 0:
        (
            board_id,
            drv1_enabled, drv1_ready, drv1_faulted,
            drv2_enabled, drv2_ready, drv2_faulted,
            drv3_enabled, drv3_ready, drv3_faulted,
            pot1, current1,
            pot2, current2,
            pot3, current3,
            servo_voltage,
        ) = parseResponse(r)
        print('board id:', board_id)
        print('drive 1:' + ' enabled' * drv1_enabled + ' disabled' * (not drv1_enabled) + ' ready' * drv1_ready + ' faulted' * drv1_faulted)
        print('drive 2:' + ' enabled' * drv2_enabled + ' disabled' * (not drv2_enabled) + ' ready' * drv2_ready + ' faulted' * drv2_faulted)
        print('drive 3:' + ' enabled' * drv3_enabled + ' disabled' * (not drv3_enabled) + ' ready' * drv3_ready + ' faulted' * drv3_faulted)
        print('pot1:', pot1)
        print('current1:', current1)
        print('pot2:', pot2)
        print('current2:', current2)
        print('pot3:', pot3)
        print('current3:', current3)
        print('servo voltage:', servo_voltage)


if __name__ == '__main__':
    import sys
    import time

    serial_port = 'COM3'#'/dev/ttyACM0'
    if len(sys.argv) >= 2:
        serial_port = sys.argv[1]
    board_id = 42
    if len(sys.argv) >= 3:
        board_id = int(sys.argv[2])

    print('opening serial device on port:', serial_port)
    with serial.Serial(serial_port, 115200, timeout=0.1) as ser:
        sendCommandAndPrintResponse(ser, makeCommand(board_id, False, True, True, 0, 0, True, True, 0, 0, True, True, 0, 0))
        time.sleep(1)
        sendCommandAndPrintResponse(ser, makeCommand(board_id, False, True, False, 0, 0, True, False, 0, 0, True, False, 0, 0))
