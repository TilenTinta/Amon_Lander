####################################################################
# File Name          : protocol.py
# Author             : Tinta T.
# Version            : V1.0.0
# Date               : 2026/02/09
# Description        : Protocol constants and frame helpers
####################################################################

from __future__ import annotations

from dataclasses import dataclass

# Frame and protocol base
SIG_SOF = 0xAA
PROTOCOL_VER = 0x01
BOOT_VER = 0x01
HEADER_SHIFT_UART = 0x08
HEADER_SHIFT_RF = 0x06

# Address / IDs
ID_PC = 0x01
ID_LINK_BOOT = 0x10
ID_LINK_SW = 0x11
ID_DRONE = 0x20
ID_BROADCAST = 0xFF

# Flags
FLAG_DATA = 0x05
FLAG_ACK = 0x01
FLAG_ERR = 0x02
FLAG_STREAM = 0x03
FLAG_FRAG = 0x04
FLAG_NAN = 0xF0

# Opcodes
OPT_PING = 0x01
OPT_NOP = 0x00
OPT_ERROR_REPORT = 0x02
OPT_ERROR_TX = 0x03
OPT_PAIR_STATUS = 0x10
OPT_PAIR_START = 0x11
OPT_TELEMETRY_STREAM = 0x12
OPT_LINK_GET_PARAMS = 0x20
OPT_LINK_SET_PARAMS = 0x21
OPT_DRONE_GET_PARAMS = 0x30
OPT_DRONE_SET_PARAMS = 0x31
OPT_DRONE_SET_STATE = 0x32
OPT_DRONE_COMMAND = 0x33
OPT_TELEMETRY = 0x40
OPT_LOG_DUMP = 0x50
OPT_LOG_RM = 0x51

# Drone state payload values for OPT_DRONE_SET_STATE
TVL_STATE_STARTUP = 0x40
TVL_STATE_IDLE = 0x41
TVL_STATE_ERROR = 0x42
TVL_STATE_ARM = 0x43
TVL_STATE_FLY = 0x44
TVL_STATE_FLY_OVER = 0x45
TVL_STATE_CALIB = 0x46

# Bootloader commands
CMD_INFO = 0x01
CMD_ERASE = 0x02
CMD_WRITE = 0x03
CMD_VERIFY = 0x04
CMD_JUMP_APP = 0x05
CMD_END_OF_FW = 0x06
CMD_ACK = 0x80
CMD_ERR = 0x81

# Bootloader response codes
CODE_BAD_CRC = 0x01
CODE_BOOT_VER = 0x02
CODE_SW_CRC = 0x02
CODE_EXIT_BOOT = 0x03
CODE_DATA_WRITEN = 0x10

# Transcode return codes
TRANSCODE_OK = 0x00
TRANSCODE_CRC_ERR = 0x01
TRANSCODE_VER_ERR = 0x02
TRANSCODE_DEST_ERR = 0x03
TRANSCODE_BROADCAST = 0x04
TRANSCODE_BOOT_PKT = 0x0A

# TLV / telemetry
TVL_BAT = 0x01
TVL_FW_VER = 0x01
TVL_RSSI = 0x02
TVL_RF_CH = 0x11
TVL_ADDR_LEN = 0x12
TVL_DATA_RATE = 0x13
TVL_R1_PWR = 0x14
TVL_R2_PWR = 0x15
TVL_RF_STREAM = 0x16
TVL_RF_TX_CNT = 0x17
TVL_RF_FAIL_CNT = 0x18

TVL_DRONE_MODE = 0x20
TVL_BAT_MAIN = 0x21
TVL_BAT_EDF = 0x21
TVL_ERR = 0x22
TVL_DATE_TIME = 0x23
TVL_TLM = 0x24
TVL_FLIGHT_MODE = 0x25

TVL_THP = 0x30
TVL_ANGL = 0x31
TVL_ALT = 0x32
TVL_IMU = 0x33
TVL_IMU_TEMP = 0x34
TVL_GPS = 0x35


# CRC-16/Modbus (poly 0xA001) with 0xFFFF initial value
def crc16_cal(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# Header: VER, FLAGS, SRC, DST, OPCODE, PLEN
def build_frame(
    *,
    version: int,
    flags: int,
    src: int,
    dst: int,
    opcode: int,
    payload: bytes = b"",
) -> bytes:
    header = bytes(
        [
            version,
            flags,
            src,
            dst,
            opcode,
            len(payload),
        ]
    )

    # LEN includes header + payload + CRC16
    length = len(header) + len(payload) + 2
    crc_input = bytes([length]) + header + payload
    crc = crc16_cal(crc_input)
    crc_lo = crc & 0xFF
    crc_hi = (crc >> 8) & 0xFF
    return bytes([SIG_SOF, length]) + header + payload + bytes([crc_lo, crc_hi])


# Default ping request frame
def build_ping_frame() -> bytes:
    return build_frame(
        version=PROTOCOL_VER,
        flags=FLAG_DATA,
        src=ID_PC,
        dst=ID_DRONE,
        opcode=OPT_PING,
        payload=b"",
    )


@dataclass
class ParsedFrame:
    version: int
    flags: int
    src: int
    dst: int
    opcode: int
    payload: bytes


# Basic length and SOF check
def parse_frame(frame: bytes) -> ParsedFrame | None:

    if len(frame) < 10 or frame[0] != SIG_SOF:
        return None
    
    length = frame[1]
    if len(frame) != 2 + length:
        return None
    
    # Validate CRC before decoding fields
    crc_recv = frame[-2] | (frame[-1] << 8)
    crc_calc = crc16_cal(frame[1:-2])
    if crc_recv != crc_calc:
        return None
    
    header = frame[2:8]
    payload_len = header[5]

    # Payload length must match LEN byte minus header + CRC
    if payload_len != length - 8:
        return None
    
    payload = frame[8 : 8 + payload_len]
    return ParsedFrame(
        version=header[0],
        flags=header[1],
        src=header[2],
        dst=header[3],
        opcode=header[4],
        payload=payload,
    )
