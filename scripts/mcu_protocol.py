#!/usr/bin/env python
import struct

HEADER = 0xA5
REQ_JOY = 0x42
RES_JOY = 0xC2
REQ_MOTOR_ONOFF = 0x21
REQ_MOTOR_OPTION_ON = 0x01
REQ_MOTOR_OPTION_OFF = 0x00
REQ_MOTOR_SET_VEL = 0x22
REQ_MOTOR_ENC = 0x23
RES_MOTOR_ENC = 0x24


def append_checksum(buff):
    checksum = 0
    for c in buff:
        checksum ^= ord(c)
    return checksum


def make_motor_on():
    req = struct.pack('<BBBB', HEADER, 0x02, REQ_MOTOR_ONOFF,
                       REQ_MOTOR_OPTION_ON)
    req += chr(append_checksum(req))
    return req


def make_motor_off():
    req = struct.pack('<BBBB', HEADER, 0x02, REQ_MOTOR_ONOFF,
                       REQ_MOTOR_OPTION_OFF)
    req += chr(append_checksum(req))
    return req


def make_req_joy():
    req = struct.pack('<BBB', HEADER, 0x01,
                          REQ_JOY)
    req += chr(append_checksum(req))
    return req


def make_motor_set_vel(vel, rot):
    req = struct.pack('<BBBhh', HEADER, 0x05,
                      REQ_MOTOR_SET_VEL, vel, rot)
    req += chr(append_checksum(req))
    return req
