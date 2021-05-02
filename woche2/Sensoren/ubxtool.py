#!/usr/bin/env python
# -*- coding: UTF-8
'''
ubxtool -- u-blox configurator and packet decoder

usage: ubxtool [OPTIONS] [server[:port[:device]]]
'''
# https://gpsd.io/ubxtool-examples.html
# This file is Copyright (c) 2018 by the GPSD project
# BSD terms apply: see the file COPYING in the distribution root for details.
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!
#
# ENVIRONMENT:
#    Options in the UBXOPTS environment variable will be parsed before
#    the CLI options.  A handy place to put your '-f /dev/ttyXX -s SPEED'
#
# To see what constellations are enabled:
#       ubxtool -p GNSS -f /dev/ttyXX
#
# To disable GALILEO and enable GALILEO:
#       ubxtool -d GLONASS -f /dev/ttyXX
#       ubxtool -e GALILEO -f /dev/ttyXX
#
# To read GPS messages a log file:
#       ubxtool -v 2 -f test/daemon/ublox-neo-m8n.log
#
# References:
#   [1] IS-GPS-200J

from __future__ import absolute_import, print_function, division

import binascii      # for binascii.hexlify()
from functools import reduce  # pylint: disable=redefined-builtin
import getopt        # for getopt.getopt(), to parse CLI options
import operator      # for or_
import os            # for os.environ
import socket        # for socket.error
import stat          # for stat.S_ISBLK()
import struct        # for pack()
import sys
import time

PROG_NAME = 'ubxtool'

try:
    import serial
except ImportError:
    serial = None  # Defer complaining until we know we need it.

try:
    import gps
except ImportError:
    # PEP8 says local imports last
    sys.stderr.write("%s: failed to import gps, check PYTHON_PATH\n" %
                     PROG_NAME)
    sys.exit(2)

gps_version = '3.19'

time.clock = time.perf_counter  # genial

VERB_QUIET = 0   # quiet
VERB_NONE = 1    # just output requested data and some info
VERB_DECODE = 2  # decode all messages
VERB_INFO = 3    # more info
VERB_RAW = 4     # raw info
VERB_PROG = 5    # program trace

# dictionary to hold all user options
opts = {
    # command to send to GPS, -c
    'command': None,
    # default -x items, up to 64 per call
    'del_item': [],
    # command for -d disable
    'disable': None,
    # command for -e enable
    'enable': None,
    # help requested
    'help': None,
    # default input -f file
    'input_file_name': None,
    # default -g items, up to 64 per call
    'get_item': [],
    # default forced wait? -W
    'input_forced_wait': False,
    # default port speed -s
    'input_speed': 9600,
    # default input wait time -w in seconds
    'input_wait': 2.0,
    # interface for port-related commands
    'interface': None,
    # optional mode to -p P
    'mode': None,
    # the name of an OAF file, extension .jpo
    'oaf_name': None,
    # poll command -p
    'poll': None,
    # protocol version for sent commands
    # u-blox 5, firmware 4 to 6 is protver 10 to 12
    # u-blox 6, firmware 6 to 7 is protver 12 to 13
    # u-blox 6, firmware 1 is protver 14
    # u-blox 7, firmware 1 is protver 14
    # u-blox 8, is protver 15 to 23
    # u-blox 9, firmware 1 is protver 27
    # u-blox F9T, firmware 2 is protver 29
    'protver': 10,
    # raw log file name
    'raw_file': None,
    # open port read only -r
    'read_only': False,
    # default -z item
    'set_item': [],
    # speed to set GPS -S
    'set_speed': None,
    # target gpsd (server:port:device) to connect to
    'target': {"server": None, "port": gps.GPSD_PORT, "device": None},
    # verbosity level, -v
    'verbosity': VERB_NONE,
    # contents of environment variable UBXOPTS
    'progopts': '',
}


# I'd like to use pypy module bitstring or bitarray, but
# people complain when non stock python modules are used here.
def unpack_s11(word, pos):
    """Grab a signed 11 bits from offset pos of word"""

    bytes = bytearray(2)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0x07
    if 0x04 & bytes[1]:
        # extend the sign
        bytes[1] |= 0xf8
    u = struct.unpack_from('<h', bytes, 0)
    return u[0]


def unpack_s11s(word):
    """Grab the weird split signed 11 bits from word"""

    newword = (word >> 22) & 0xff
    newword <<= 3
    newword |= (word >> 8) & 0x07
    return unpack_s11(newword, 0)


def unpack_s14(word, pos):
    """Grab a signed 14 bits from offset pos of word"""

    bytes = bytearray(2)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0x3f
    if 0x20 & bytes[1]:
        # extend the sign
        bytes[1] |= 0xc0
    u = struct.unpack_from('<h', bytes, 0)
    return u[0]


def unpack_s16(word, pos):
    """Grab a signed two bytes from offset pos of word"""

    bytes = bytearray(2)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0xff
    u = struct.unpack_from('<h', bytes, 0)
    return u[0]


def unpack_u16(word, pos):
    """Grab a unsigned two bytes from offset pos of word"""

    bytes = bytearray(2)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0xff
    u = struct.unpack_from('<H', bytes, 0)
    return u[0]


def unpack_s22(word, pos):
    """Grab a signed 22 bits from offset pos of word"""

    bytes = bytearray(4)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0xff
    bytes[2] = (word >> (pos + 16)) & 0x3f
    bytes[3] = 0
    if 0x20 & bytes[2]:
        # extend the sign
        bytes[2] |= 0xc0
        bytes[3] = 0xff

    u = struct.unpack_from('<l', bytes, 0)
    return u[0]


def unpack_s24(word, pos):
    """Grab a signed 24 bits from offset pos of word"""

    bytes = bytearray(4)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0xff
    bytes[2] = (word >> (pos + 16)) & 0xff
    bytes[3] = 0
    if 0x80 & bytes[2]:
        # extend the sign
        bytes[3] = 0xff

    u = struct.unpack_from('<l', bytes, 0)
    return u[0]


def unpack_u24(word, pos):
    """Grab an unsigned 24 bits from offset pos of word"""

    bytes = bytearray(4)
    bytes[0] = (word >> pos) & 0xff
    bytes[1] = (word >> (pos + 8)) & 0xff
    bytes[2] = (word >> (pos + 16)) & 0xff
    bytes[3] = 0

    u = struct.unpack_from('<L', bytes, 0)
    return u[0]


def unpack_s32s(word, word1):
    """Grab an signed 32 bits from weird split word, word1"""

    bytes = bytearray(4)
    bytes[0] = (word >> 6) & 0xff
    bytes[1] = (word >> 14) & 0xff
    bytes[2] = (word >> 22) & 0xff
    bytes[3] = (word1 >> 6) & 0xff

    u = struct.unpack_from('<l', bytes, 0)
    return u[0]


def unpack_u32s(word, word1):
    """Grab an unsigned 32 bits from weird split word, word1"""

    bytes = bytearray(4)
    bytes[0] = (word >> 6) & 0xff
    bytes[1] = (word >> 14) & 0xff
    bytes[2] = (word >> 22) & 0xff
    bytes[3] = (word1 >> 6) & 0xff

    u = struct.unpack_from('<L', bytes, 0)
    return u[0]


def unpack_s8(word, pos):
    """Grab a signed byte from offset pos of word"""

    bytes = bytearray(1)
    bytes[0] = (word >> pos) & 0xff
    u = struct.unpack_from('<b', bytes, 0)
    return u[0]


def unpack_u8(word, pos):
    """Grab an unsigned byte from offset pos of word"""

    bytes = bytearray(1)
    bytes[0] = (word >> pos) & 0xff
    u = struct.unpack_from('<B', bytes, 0)
    return u[0]


def flag_s(flag, descs):
    """Decode flag using descs, return a string.  Ignores unknown bits."""

    s = ''
    for key, value in sorted(descs.items()):
        if key == (key & flag):
            s += value
            s += ' '

    return s.strip()


def index_s(index, descs):
    """Decode flag using descs, return a string.  Otherwise Unk"""

    if index in descs:
        s = descs[index]
    else:
        s = 'Unk'

    return s


class ubx(object):
    """class to hold u-blox stuff"""

    # when a statement identifier is received, it is stored here
    last_statement_identifier = None
    # expected statement identifier.
    expect_statement_identifier = False

    def __init__(self):
        pass

    # allowable speeds
    speeds = (460800, 230400, 153600, 115200, 57600, 38400, 19200, 9600,
              4800, 2400, 1200, 600, 300)

    # UBX Satellite Numbering
    gnss_id = {0: 'GPS',
               1: 'SBAS',
               2: 'Galileo',
               3: 'BeiDou',
               4: 'IMES',
               5: 'QZSS',
               6: 'GLONASS'}

    # Names for portID values in UBX-CFG-PRT, UBX-MON-IO, etc.
    port_ids = {0: 'DDC',  # The inappropriate name for i2c used in the spec
                1: 'UART1',
                2: 'UART2',
                3: 'USB',
                4: 'SPI',
                }
    port_id_map = dict([x[::-1] for x in port_ids.items()])
    port_id_map['UART'] = port_id_map['UART1']  # Accept synonym
    port_ids[5] = 'Reserved'  # Don't include this in port_id_map

    # Names for portID values in UBX-CFG-COMMS
    # the doc is byteswapped from here
    port_ids1 = {0: 'DDC',
                 0x100: 'UART1',
                 0x101: 'UNKa',      # seen on ZED-M9T, undocumented
                 0x200: 'UNKb',      # seen on ZED-M9T, undocumented
                 0x201: 'UART2',
                 0x300: 'USB',
                 0x400: 'SPI',
                 }
    # u-blox 9 cfg items as a 5-tuple
    # 1 - Name
    # 2 - key id
    # 3 - value type
    # 4 - scale
    # 5 - Unit
    # 6 - Description
    cfgs = (
        # CFG-GEOFENCE-
        ("CFG-GEOFENCE-CONFLVL", 0x20240011, "E1", 1, "",
         "Required confidence level for state evaluation"),
        ("CFG-GEOFENCE-USE_PIO", 0x10240012, "L", 1, "",
         "Use PIO combined fence state output"),
        ("CFG-GEOFENCE-PINPOL", 0x20240013, "E1", 1, "",
         "PIO pin polarity"),
        ("CFG-GEOFENCE-PIN", 0x20240014, "U1", 1, "",
         "PIO pin number"),
        ("CFG-GEOFENCE-USE_FENCE1", 0x10240020, "L", 1, "",
         "Use first geofence"),
        ("CFG-GEOFENCE-FENCE1_LAT", 0x40240021, "I4", 1e-7, "deg",
         "Latitude of the first geofence circle center"),
        ("CFG-GEOFENCE-FENCE1_LON", 0x40240022, "I4", 1e-7, "deg",
         "Longitude of the first geofence circle center"),
        ("CFG-GEOFENCE-FENCE1_RAD", 0x40240023, "U4", 0.01, "m",
         "Radius of the first geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE2", 0x10240030, "L", 1, "",
         "Use second geofence"),
        ("CFG-GEOFENCE-FENCE2_LAT", 0x40240031, "I4", 1e-7, "deg",
         "Latitude of the second geofence circle center"),
        ("CFG-GEOFENCE-FENCE2_LON", 0x40240032, "I4", 1e-7, "deg",
         "Longitude of the second geofence circle center"),
        ("CFG-GEOFENCE-FENCE2_RAD", 0x40240033, "U4", 0.01, "m",
         "Radius of the second geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE3", 0x10240040, "L", 1, "",
         "Use third geofence"),
        ("CFG-GEOFENCE-FENCE3_LAT", 0x40240041, "I4", 1e-7, "deg",
         "Latitude of the third geofence circle center"),
        ("CFG-GEOFENCE-FENCE3_LON", 0x40240042, "I4", 1e-7, "deg",
         "Longitude of the third geofence circle center"),
        ("CFG-GEOFENCE-FENCE3_RAD", 0x40240043, "U4", 0.01, "m",
         "Radius of the third geofence circle"),
        ("CFG-GEOFENCE-USE_FENCE4", 0x10240050, "L", 1, "",
         "Use fourth geofence"),
        ("CFG-GEOFENCE-FENCE4_LAT", 0x40240051, "I4", 1e-7, "deg",
         "Latitude of the fourth geofence circle center"),
        ("CFG-GEOFENCE-FENCE4_LON", 0x40240052, "I4", 1e-7, "deg",
         "Longitude of the fourth geofence circle center"),
        ("CFG-GEOFENCE-FENCE4_RAD", 0x40240053, "U4", 0.01, "m",
         "Radius of the fourth geofence circle"),
        # CFG-HW
        ("CFG-HW-ANT_CFG_VOLTCTRL", 0x10a3002e, "L", 1, "",
         "Active antenna voltage control flag"),
        ("CFG-HW-ANT_CFG_SHORTDET", 0x10a3002f, "L", 1, "",
         "Short antenna detection flag"),
        ("CFG-HW-ANT_CFG_SHORTDET_POL", 0x10a30030, "L", 1, "",
         "Short antenna detection polarity"),
        ("CFG-HW-ANT_CFG_OPENDET", 0x10a30031, "L", 1, "",
         "Open antenna detection flag"),
        ("CFG-HW-ANT_CFG_OPENDET_POL", 0x10a30032, "L", 1, "",
         "Open antenna detection polarity"),
        ("CFG-HW-ANT_CFG_PWRDOWN", 0x10a30033, "L", 1, "",
         "Power down antenna flag"),
        ("CFG-HW-ANT_CFG_PWRDOWN_POL", 0x10a30034, "L", 1, "",
         "Power down antenna logic polarity"),
        ("CFG-HW-ANT_CFG_RECOVER", 0x10a30035, "L", 1, "",
         "Automatic recovery from short state flag"),
        ("CFG-HW-ANT_SUP_SWITCH_PIN", 0x20a30036, "U1", 1, "",
         "ANT1 PIO number"),
        ("CFG-HW-ANT_SUP_SHORT_PIN", 0x20a30037, "U1", 1, "",
         "ANT0 PIO number"),
        ("CFG-HW-ANT_SUP_OPEN_PIN", 0x20a30038, "U1", 1, "",
         "ANT2 PIO number"),
        # CFG-I2C
        ("CFG-I2C-ADDRESS", 0x20510001, "U1", 1, "",
         "I2C slave address of the receiver"),
        ("CFG-I2C-EXTENDEDTIMEOUT", 0x10510002, "L", 1, "",
         "Flag to disable timeouting the interface after 1.5 s"),
        ("CFG-I2C-ENABLED", 0x10510003, "L", 1, "",
         "Flag to indicate if the I2C interface should be enabled"),
        # CFG-I2CINPROT
        ("CFG-I2CINPROT-UBX", 0x10710001, "L", 1, "",
         "Flag to indicate if UBX should be an input on I2C"),
        ("CFG-I2CINPROT-NMEA", 0x10710002, "L", 1, "",
         "Flag to indicate if NMEA should be an input on I2C"),
        ("CFG-I2CINPROT-RTCM2X", 0x10710003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input on I2C"),
        ("CFG-I2CINPROT-RTCM3X", 0x10710004, "L", 1, "",
         "Flag to indicate if RTCM3X should be input on I2C"),
        # CFG-I2COUTPROT
        ("CFG-I2COUTPROT-UBX", 0x10720001, "L", 1, "",
         "Flag to indicate if UBX should be an output on I2C"),
        ("CFG-I2COUTPROT-NMEA", 0x10720002, "L", 1, "",
         "Flag to indicate if NMEA should be an output on I2C"),
        ("CFG-I2COUTPROT-RTCM3X", 0x10720004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output on I2C"),
        # CFG-INFMSG-
        ("CFG-INFMSG-UBX_I2C", 0x20920001, "X1", 1, "",
         "Information message enable flags for UBX protocol on I2C"),
        ("CFG-INFMSG-UBX_UART1", 0x20920002, "X1", 1, "",
         "Information message enable flags for UBX protocol on UART1"),
        ("CFG-INFMSG-UBX_UART2", 0x20920003, "X1", 1, "",
         "Information message enable flags for UBX protocol on UART2"),
        ("CFG-INFMSG-UBX_USB", 0x20920004, "X1", 1, "",
         "Information message enable flags for UBX protocol on USB"),
        ("CFG-INFMSG-UBX_SPI", 0x20920005, "X1", 1, "",
         "Information message enable flags for UBX protocol on SPI"),
        ("CFG-INFMSG-NMEA_I2C", 0x20920006, "X1", 1, "",
         "Information message enable flags for NMEA protocol on I2C"),
        ("CFG-INFMSG-NMEA_UART1", 0x20920007, "X1", 1, "",
         "Information message enable flags for NMEA protocol on UART1"),
        ("CFG-INFMSG-NMEA_UART2", 0x20920008, "X1", 1, "",
         "Information message enable flags for NMEA protocol on UART2"),
        ("CFG-INFMSG-NMEA_USB", 0x20920009, "X1", 1, "",
         "Information message enable flags for NMEA protocol on USB"),
        ("CFG-INFMSG-NMEA_SPI", 0x2092000a, "X1", 1, "",
         "Information message enable flags for NMEA protocol on SPI"),
        # CFG-ITFM-
        ("CFG-ITFM-BBTHRESHOLD", 0x20410001, "U1", 1, "",
         "Broadband jamming detection threshold"),
        ("CFG-ITFM-CWTHRESHOLD", 0x20410002, "U1", 1, "",
         "CW jamming detection threshold"),
        ("CFG-ITFM-ENABLE", 0x1041000d, "L", 1, "",
         "Enable interference detection"),
        ("CFG-ITFM-ANTSETTING", 0x20410010, "E1", 1, "",
         "Antenna setting"),
        ("CFG-ITFM-ENABLE_AUX", 0x10410013, "L", 1, "",
         "Set to true to scan auxiliary bands"),
        # CFG-LOGFILTER-
        ("CFG-LOGFILTER-RECORD_ENA", 0x10de0002, "L", 1, "",
         "Recording enabled"),
        ("CFG-LOGFILTER-ONCE_PER_WAKE_UP_ENA", 0x10de0003, "L", 1, "",
         "Once per wakeup"),
        ("CFG-LOGFILTER-APPLY_ALL_FILTERS", 0x10de0004, "L", 1, "",
         "Apply all filter settings"),
        ("CFG-LOGFILTER-MIN_INTERVAL", 0x30de0005, "U2", 1, "s",
         "Minimum time interval between logged positions"),
        ("CFG-LOGFILTER-TIME_THRS", 0x30de0006, "U2", 1, "s",
         "Time threshold"),
        ("CFG-LOGFILTER-SPEED_THRS", 0x30de0007, "U2", 1, "m/s",
         "Speed threshold"),
        ("CFG-LOGFILTER-POSITION_THRS", 0x40de0008, "U4", 1, "m",
         "Position threshold"),
        # CFG-MOT-
        ("CFG-MOT-GNSSSPEED_THRS", 0x20250038, "U1", 0.01, "m/s",
         "GNSS speed threshold below which platform is considered "
         "as stationary"),
        ("CFG-MOT-GNSSDIST_THRS", 0x3025003b, "U2", 1, "",
         "Distance above which GNSS-based stationary motion is exit"),
        # CFG-MSGOUT-NMEA
        ("CFG-MSGOUT-NMEA_ID_DTM_I2C", 0x209100a6, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_DTM_SPI", 0x209100aa, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_DTM_UART1", 0x209100a7, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_DTM_UART2", 0x209100a8, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_DTM_USB", 0x209100a9, "U1", 1, "",
         "Output rate of the NMEA-GX-DTM message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GBS_I2C", 0x209100dd, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GBS_SPI", 0x209100e1, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GBS_UART1", 0x209100de, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GBS_UART2", 0x209100df, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GBS_USB", 0x209100e0, "U1", 1, "",
         "Output rate of the NMEA-GX-GBS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GGA_I2C", 0x209100ba, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GGA_SPI", 0x209100be, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GGA_UART1", 0x209100bb, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GGA_UART2", 0x209100bc, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GGA_USB", 0x209100bd, "U1", 1, "",
         "Output rate of the NMEA-GX-GGA message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GLL_I2C", 0x209100c9, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GLL_SPI", 0x209100cd, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GLL_UART1", 0x209100ca, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GLL_UART2", 0x209100cb, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GLL_USB", 0x209100cc, "U1", 1, "",
         "Output rate of the NMEA-GX-GLL message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GNS_I2C", 0x209100b5, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GNS_SPI", 0x209100b9, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GNS_UART1", 0x209100b6, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GNS_UART2", 0x209100b7, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GNS_USB", 0x209100b8, "U1", 1, "",
         "Output rate of the NMEA-GX-GNS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GRS_I2C", 0x209100ce, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GRS_SPI", 0x209100d2, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GRS_UART1", 0x209100cf, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GRS_UART2", 0x209100d0, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GRS_USB", 0x209100d1, "U1", 1, "",
         "Output rate of the NMEA-GX-GRS message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GSA_I2C", 0x209100bf, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GSA_SPI", 0x209100c3, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GSA_UART1", 0x209100c0, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GSA_UART2", 0x209100c1, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GSA_USB", 0x209100c2, "U1", 1, "",
         "Output rate of the NMEA-GX-GSA message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GST_I2C", 0x209100d3, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GST_SPI", 0x209100d7, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GST_UART1", 0x209100d4, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GST_UART2", 0x209100d5, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_GST_USB", 0x209100d6, "U1", 1, "",
         "Output rate of the NMEA-GX-GST message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_GSV_I2C", 0x209100c4, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_GSV_SPI", 0x209100c8, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_GSV_UART1", 0x209100c5, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_GSV_UART2", 0x209100c6, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port UART"),
        ("CFG-MSGOUT-NMEA_ID_GSV_USB", 0x209100c7, "U1", 1, "",
         "Output rate of the NMEA-GX-GSV message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_RMC_I2C", 0x209100ab, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_RMC_SPI", 0x209100af, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_RMC_UART1", 0x209100ac, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_RMC_UART2", 0x209100ad, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_RMC_USB", 0x209100ae, "U1", 1, "",
         "Output rate of the NMEA-GX-RMC message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_VLW_I2C", 0x209100e7, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_VLW_SPI", 0x209100eb, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_VLW_UART1", 0x209100e8, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_VLW_UART2", 0x209100e9, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_VLW_USB", 0x209100ea, "U1", 1, "",
         "Output rate of the NMEA-GX-VLW message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_VTG_I2C", 0x209100b0, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_VTG_SPI", 0x209100b4, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_VTG_UART1", 0x209100b1, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_VTG_UART2", 0x209100b2, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_VTG_USB", 0x209100b3, "U1", 1, "",
         "Output rate of the NMEA-GX-VTG message on port USB"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_I2C", 0x209100d8, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port I2C"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_SPI", 0x209100dc, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port SPI"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_UART1", 0x209100d9, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port UART1"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_UART2", 0x209100da, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port UART2"),
        ("CFG-MSGOUT-NMEA_ID_ZDA_USB", 0x209100db, "U1", 1, "",
         "Output rate of the NMEA-GX-ZDA message on port USB"),
        # CFG-MSGOUT-PUBX
        ("CFG-MSGOUT-PUBX_ID_POLYP_I2C", 0x209100ec, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_SPI", 0x209100f0, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port SPI"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_UART1", 0x209100ed, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_UART2", 0x209100ee, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYP_USB", 0x209100ef, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX00 message on port USB"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_I2C", 0x209100f1, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_SPI", 0x209100f5, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port SPI"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_UART1", 0x209100f2, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_UART2", 0x209100f3, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYS_USB", 0x209100f4, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX03 message on port USB"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_I2C", 0x209100f6, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port I2C"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_SPI", 0x209100fa, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port SPI"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_UART1", 0x209100f7, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port UART1"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_UART2", 0x209100f8, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port UART2"),
        ("CFG-MSGOUT-PUBX_ID_POLYT_USB", 0x209100f9, "U1", 1, "",
         "Output rate of the NMEA-GX-PUBX04 message on port USB"),
        # CFG-MSGOUT-RTCM_3X
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_I2C", 0x209102bd, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_SPI", 0x209102c1, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_UART1", 0x209102be, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_UART2", 0x209102bf, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1005_USB", 0x209102c0, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1005 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_I2C", 0x2091035e, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_SPI", 0x20910362, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_UART1", 0x2091035f, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_UART2", 0x20910360, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1074_USB", 0x20910361, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1074 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_I2C", 0x209102cc, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port I2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_SPI", 0x209102d0, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_UART1", 0x209102cd, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_UART2", 0x209102ce, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1077_USB", 0x209102cf, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1077 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_I2C", 0x209102d1, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_SPI", 0x20910367, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_UART1", 0x20910364, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_UART2", 0x20910365, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1084_USB", 0x20910366, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1084 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_SPI", 0x209102d5, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_UART1", 0x209102d2, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_UART2", 0x209102d3, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1087_USB", 0x209102d4, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1087 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_I2C", 0x20910368, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_SPI", 0x2091036c, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_UART1", 0x20910369, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_UART2", 0x2091036a, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1094_USB", 0x2091036b, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1094 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_I2C", 0x20910318, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_SPI", 0x2091031c, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_UART1", 0x20910319, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_UART2", 0x2091031a, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1097_USB", 0x2091031b, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1097 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_I2C", 0x2091036d, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_SPI", 0x20910371, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_UART1", 0x2091036e, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_UART2", 0x2091036f, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1124_USB", 0x20910370, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1124 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_I2C", 0x209102d6, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_SPI", 0x209102da, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_UART1", 0x209102d7, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_UART2", 0x209102d8, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1127_USB", 0x209102d9, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1127 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_I2C", 0x20910303, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_SPI", 0x20910307, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_UART1", 0x20910304, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_UART2", 0x20910305, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE1230_USB", 0x20910306, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE1230 message on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_I2C", 0x209102fe, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 0 message "
         "on port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_SPI", 0x20910302, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 0 message "
         "on port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART1", 0x209102ff, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 0 message "
         "on port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2", 0x20910300, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 0 message "
         "on port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB", 0x20910301, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 0 message "
         "on port USB"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_I2C", 0x20910381, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 1 message on  "
         "port I2C"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_SPI", 0x20910385, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 1 message on  "
         "port SPI"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART1", 0x20910382, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 1 message on  "
         "port UART1"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART2", 0x20910383, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 1 message on "
         " port UART2"),
        ("CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB", 0x20910384, "U1", 1, "",
         "Output rate of the RTCM-3X-TYPE4072, sub-type 1 message "
         "on port USB"),
        # CFG-MSGOUT-UBX_LOG
        ("CFG-MSGOUT-UBX_LOG_INFO_I2C", 0x20910259, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port I2C"),
        ("CFG-MSGOUT-UBX_LOG_INFO_SPI", 0x2091025d, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port SPI"),
        ("CFG-MSGOUT-UBX_LOG_INFO_UART1", 0x2091025a, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port UART1"),
        ("CFG-MSGOUT-UBX_LOG_INFO_UART2", 0x2091025b, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port UART2"),
        ("CFG-MSGOUT-UBX_LOG_INFO_USB", 0x2091025c, "U1", 1, "",
         "Output rate of the UBX-LOG-INFO message on port USB"),
        # CFG-MSGOUT-UBX_MON
        ("CFG-MSGOUT-UBX_MON_COMMS_I2C", 0x2091034f, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_COMMS_SPI", 0x20910353, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_COMMS_UART1", 0x20910350, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_COMMS_UART2", 0x20910351, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_COMMS_USB", 0x20910352, "U1", 1, "",
         "Output rate of the UBX-MON-COMMS message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW2_I2C", 0x209101b9, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW2_SPI", 0x209101bd, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_HW2_UART1", 0x209101ba, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW2_UART2", 0x209101bb, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW2_USB", 0x209101bc, "U1", 1, "",
         "Output rate of the UBX-MON-HW2 message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW3_I2C", 0x20910354, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW3_SPI", 0x20910358, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_HW3_UART1", 0x20910355, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW3_UART2", 0x20910356, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW3_USB", 0x20910357, "U1", 1, "",
         "Output rate of the UBX-MON-HW3 message on port USB"),
        ("CFG-MSGOUT-UBX_MON_HW_I2C", 0x209101b4, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_HW_SPI", 0x209101b8, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_HW_UART1", 0x209101b5, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_HW_UART2", 0x209101b6, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_HW_USB", 0x209101b7, "U1", 1, "",
         "Output rate of the UBX-MON-HW message on port USB"),
        ("CFG-MSGOUT-UBX_MON_IO_I2C", 0x209101a5, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_IO_SPI", 0x209101a9, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_IO_UART1", 0x209101a6, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_IO_UART2", 0x209101a7, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_IO_USB", 0x209101a8, "U1", 1, "",
         "Output rate of the UBX-MON-IO message on port USB"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_I2C", 0x20910196, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_SPI", 0x2091019a, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_UART1", 0x20910197, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_UART2", 0x20910198, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_MSGPP_USB", 0x20910199, "U1", 1, "",
         "Output rate of the UBX-MON-MSGPP message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RF_I2C", 0x20910359, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RF_SPI", 0x2091035d, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_RF_UART1", 0x2091035a, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RF_UART2", 0x2091035b, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RF_USB", 0x2091035c, "U1", 1, "",
         "Output rate of the UBX-MON-RF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_I2C", 0x209101a0, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_SPI", 0x209101a4, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_UART1", 0x209101a1, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_UART2", 0x209101a2, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RXBUF_USB", 0x209101a3, "U1", 1, "",
         "Output rate of the UBX-MON-RXBUF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_RXR_I2C", 0x20910187, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_RXR_SPI", 0x2091018b, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_RXR_UART1", 0x20910188, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_RXR_UART2", 0x20910189, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_RXR_USB", 0x2091018a, "U1", 1, "",
         "Output rate of the UBX-MON-RXR message on port USB"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_I2C", 0x2091019b, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_SPI", 0x2091019f, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART1", 0x2091019c, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART2", 0x2091019d, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_USB", 0x2091019e, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port USB"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_I2C", 0x2091019b, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port I2C"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_SPI", 0x2091019f, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port SPI"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART1", 0x2091019c, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART1"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_UART2", 0x2091019d, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port UART2"),
        ("CFG-MSGOUT-UBX_MON_TXBUF_USB", 0x2091019e, "U1", 1, "",
         "Output rate of the UBX-MON-TXBUF message on port USB"),
        # CFG-MSGOUT-UBX_NAV
        ("CFG-MSGOUT-UBX_NAV_CLOCK_I2C", 0x20910065, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_SPI", 0x20910069, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_UART1", 0x20910066, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_UART2", 0x20910067, "U1", 1, "",
         "Output rate of the UBX-NAV-CLOCK message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_CLOCK_USB", 0x20910068, "U1", 1, "",
         "Output rate of the UBX-NAV- CLOCK message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_DOP_I2C", 0x20910038, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_DOP_SPI", 0x2091003c, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_DOP_UART1", 0x20910039, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_DOP_UART2", 0x2091003a, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_DOP_USB", 0x2091003b, "U1", 1, "",
         "Output rate of the UBX-NAV-DOP message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_EOE_I2C", 0x2091015f, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_EOE_SPI", 0x20910163, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_EOE_UART1", 0x20910160, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_EOE_UART2", 0x20910161, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_EOE_USB", 0x20910162, "U1", 1, "",
         "Output rate of the UBX-NAV-EOE message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_I2C", 0x209100a1, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_SPI", 0x209100a5, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_UART1", 0x209100a2, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_UART2", 0x209100a3, "U1", 1, "",
         "Output rate of the UBX-NAV-GEOFENCE message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_GEOFENCE_USB", 0x209100a4, "U1", 1, "",
         "Output rate of the UBX-NAV- GEOFENCE message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_I2C", 0x2091002e, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_SPI", 0x20910032, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART1", 0x2091002f, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART2", 0x20910030, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSECEF_USB", 0x20910031, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_I2C", 0x20910033, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_SPI", 0x20910037, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1", 0x20910034, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART2", 0x20910035, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB", 0x20910036, "U1", 1, "",
         "Output rate of the UBX-NAV-HPPOSLLH message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_ODO_I2C", 0x2091007e, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_ODO_SPI", 0x20910082, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_ODO_UART1", 0x2091007f, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_ODO_UART2", 0x20910080, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_ODO_USB", 0x20910081, "U1", 1, "",
         "Output rate of the UBX-NAV-ODO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_ORB_I2C", 0x20910010, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_ORB_SPI", 0x20910014, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_ORB_UART1", 0x20910011, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_ORB_UART2", 0x20910012, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_ORB_USB", 0x20910013, "U1", 1, "",
         "Output rate of the UBX-NAV-ORB message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_I2C", 0x20910024, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_SPI", 0x20910028, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_UART1", 0x20910025, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_UART2", 0x20910026, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_POSECEF_USB", 0x20910027, "U1", 1, "",
         "Output rate of the UBX-NAV-POSECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_I2C", 0x20910029, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_SPI", 0x2091002d, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_UART1", 0x2091002a, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_UART2", 0x2091002b, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_POSLLH_USB", 0x2091002c, "U1", 1, "",
         "Output rate of the UBX-NAV-POSLLH message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_PVT_I2C", 0x20910006, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_PVT_SPI", 0x2091000a, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_PVT_UART1", 0x20910007, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_PVT_UART2", 0x20910008, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_PVT_USB", 0x20910009, "U1", 1, "",
         "Output rate of the UBX-NAV-PVT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_I2C", 0x2091008d, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_SPI", 0x20910091, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1", 0x2091008e, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_UART2", 0x2091008f, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 0x20910090, "U1", 1, "",
         "Output rate of the UBX-NAV-RELPOSNED message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SAT_I2C", 0x20910015, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SAT_SPI", 0x20910019, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SAT_UART1", 0x20910016, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SAT_UART2", 0x20910017, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SAT_USB", 0x20910018, "U1", 1, "",
         "Output rate of the UBX-NAV-SAT message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_I2C", 0x2091006a, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_SPI", 0x2091006e, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_UART1", 0x2091006b, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_UART2", 0x2091006c, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SBAS_USB", 0x2091006d, "U1", 1, "",
         "Output rate of the UBX-NAV-SBAS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SIG_I2C", 0x20910345, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SIG_SPI", 0x20910349, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SIG_UART1", 0x20910346, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SIG_UART2", 0x20910347, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SIG_USB", 0x20910348, "U1", 1, "",
         "Output rate of the UBX-NAV-SIG message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_I2C", 0x2091001a, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_SPI", 0x2091001e, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_UART1", 0x2091001b, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_UART2", 0x2091001c, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_STATUS_USB", 0x2091001d, "U1", 1, "",
         "Output rate of the UBX-NAV-STATUS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_I2C", 0x20910088, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_SPI", 0x2091008c, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_UART1", 0x20910089, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_UART2", 0x2091008a, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_SVIN_USB", 0x2091008b, "U1", 1, "",
         "Output rate of the UBX-NAV-SVIN message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_I2C", 0x20910051, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI", 0x20910055, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_UART1", 0x20910052, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_UART2", 0x20910053, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEBDS_USB", 0x20910054, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEBDS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_I2C", 0x20910056, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI", 0x2091005a, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_UART1", 0x20910057, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_UART2", 0x20910058, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGAL_USB", 0x20910059, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGAL message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_I2C", 0x2091004c, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_SPI", 0x20910050, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_UART1", 0x2091004d, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_UART2", 0x2091004e, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGLO_USB", 0x2091004f, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGLO message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_I2C", 0x20910047, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI", 0x2091004b, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1", 0x20910048, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_UART2", 0x20910049, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEGPS_USB", 0x2091004a, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEGPS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_I2C", 0x20910060, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_SPI", 0x20910064, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_UART1", 0x20910061, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_UART2", 0x20910062, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMELS_USB", 0x20910063, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMELS message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_I2C", 0x2091005b, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI", 0x2091005f, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port S"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1", 0x2091005c, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_UART2", 0x2091005d, "U1", 1, "",
         "Output rate of the UBX-NAV-TIMEUTC message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_TIMEUTC_USB", 0x2091005e, "U1", 1, "",
         "Output rate of the UBX-NAV- TIMEUTC message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_I2C", 0x2091003d, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_SPI", 0x20910041, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_UART1", 0x2091003e, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_UART2", 0x2091003f, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_VELECEF_USB", 0x20910040, "U1", 1, "",
         "Output rate of the UBX-NAV-VELECEF message on port USB"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_I2C", 0x20910042, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port I2C"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_SPI", 0x20910046, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port SPI"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_UART1", 0x20910043, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port UART1"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_UART2", 0x20910044, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port UART2"),
        ("CFG-MSGOUT-UBX_NAV_VELNED_USB", 0x20910045, "U1", 1, "",
         "Output rate of the UBX-NAV-VELNED message on port USB"),
        # CFG-MSGOUT-UBX_RXM
        ("CFG-MSGOUT-UBX_RXM_MEASX_I2C", 0x20910204, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_SPI", 0x20910208, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_UART1", 0x20910205, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_UART2", 0x20910206, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_MEASX_USB", 0x20910207, "U1", 1, "",
         "Output rate of the UBX-RXM-MEASX message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_I2C", 0x209102a4, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_SPI", 0x209102a8, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_UART1", 0x209102a5, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_UART2", 0x209102a6, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RAWX_USB", 0x209102a7, "U1", 1, "",
         "Output rate of the UBX-RXM-RAWX message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RLM_I2C", 0x2091025e, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RLM_SPI", 0x20910262, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RLM_UART1", 0x2091025f, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_RLM_UART2", 0x20910260, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RLM_USB", 0x20910261, "U1", 1, "",
         "Output rate of the UBX-RXM-RLM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_I2C", 0x20910268, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_SPI", 0x2091026c, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_UART1", 0x20910269, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_UART2", 0x2091026a, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_RTCM_USB", 0x2091026b, "U1", 1, "",
         "Output rate of the UBX-RXM-RTCM message on port USB"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_I2C", 0x20910231, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port I2C"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_SPI", 0x20910235, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port SPI"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_UART1", 0x20910232, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port UART1"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_UART2", 0x20910233, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port UART2"),
        ("CFG-MSGOUT-UBX_RXM_SFRBX_USB", 0x20910234, "U1", 1, "",
         "Output rate of the UBX-RXM-SFRBX message on port USB"),
        # CFG-MSGOUT-UBX_TIM
        ("CFG-MSGOUT-UBX_TIM_SVIN_I2C", 0x20910097, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_SPI", 0x2091009b, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_UART1", 0x20910098, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_UART2", 0x20910099, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_SVIN_USB", 0x2091009a, "U1", 1, "",
         "Output rate of the UBX-TIM-SVIN message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_TM2_I2C", 0x20910178, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_TM2_SPI", 0x2091017c, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_TM2_UART1", 0x20910179, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_TM2_UART2", 0x2091017a, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_TM2_USB", 0x2091017b, "U1", 1, "",
         "Output rate of the UBX-TIM-TM2 message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_TP_I2C", 0x2091017d, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_TP_SPI", 0x20910181, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_TP_UART1", 0x2091017e, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_TP_UART2", 0x2091017f, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_TP_USB", 0x20910180, "U1", 1, "",
         "Output rate of the UBX-TIM-TP message on port USB"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_I2C", 0x20910092, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port I2C"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_SPI", 0x20910096, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port SPI"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_UART1", 0x20910093, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port UART1"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_UART2", 0x20910094, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port UART2"),
        ("CFG-MSGOUT-UBX_TIM_VRFY_USB", 0x20910095, "U1", 1, "",
         "Output rate of the UBX-TIM-VRFY message on port USB"),
        # CFG-NAVHPG-
        ("CFG-NAVHPG-DGNSSMODE", 0x20140011, "E1", 1, "",
         "Differential corrections mode"),
        # CFG-NAVSPG-
        ("CFG-NAVSPG-FIXMODE", 0x20110011, "E1", 1, "",
         "Position fix mode"),
        ("CFG-NAVSPG-INIFIX3D", 0x10110013, "L", 1, "",
         "Initial fix must be a 3d fix"),
        ("CFG-NAVSPG-WKNROLLOVER", 0x30110017, "U2", 1, "",
         "GPS week rollover number"),
        ("CFG-NAVSPG-USE_PPP", 0x10110019, "L", 1, "",
         "Use Precise Point Positioning"),
        ("CFG-NAVSPG-UTCSTANDARD", 0x2011001c, "E1", 1, "",
         "UTC standard to be used"),
        ("CFG-NAVSPG-DYNMODEL", 0x20110021, "E1", 1, "",
         "Dynamic platform model"),
        ("CFG-NAVSPG-ACKAIDING", 0x10110025, "L", 1, "",
         "Acknowledge assistance input messages"),
        ("CFG-NAVSPG-USE_USRDAT", 0x10110061, "L", 1, "",
         "Use user geodetic datum"),
        ("CFG-NAVSPG-USRDAT_MAJA", 0x50110062, "R8", 1, "m",
         "Geodetic datum semi-major axis"),
        ("CFG-NAVSPG-USRDAT_FLAT", 0x50110063, "R8", 1, "",
         "Geodetic datum 1.0 / flattening"),
        ("CFG-NAVSPG-USRDAT_DX", 0x40110064, "R4", 1, "m",
         "Geodetic datum X axis shift at the orgin"),
        ("CFG-NAVSPG-USRDAT_DY", 0x40110065, "R4", 1, "m",
         "Geodetic datum Y axis shift at the origin"),
        ("CFG-NAVSPG-USRDAT_DZ", 0x40110066, "R4", 1, "m",
         "Geodetic datum Z axis shift at the origin"),
        ("CFG-NAVSPG-USRDAT_ROTX", 0x40110067, "R4", 1, "arcsec",
         "Geodetic datum rotation about the X axis"),
        ("CFG-NAVSPG-USRDAT_ROTY", 0x40110068, "R4", 1, "arcsec",
         "Geodetic datum rotation about the Y axis ()"),
        ("CFG-NAVSPG-USRDAT_ROTZ", 0x40110069, "R4", 1, "arcsec",
         "Geodetic datum rotation about the Z axis"),
        ("CFG-NAVSPG-USRDAT_SCALE", 0x4011006a, "R4", 1, "ppm",
         "Geodetic datum scale factor"),
        ("CFG-NAVSPG-INFIL_MINSVS", 0x201100a1, "U1", 1, "",
         "Minimum number of satellites for navigation"),
        ("CFG-NAVSPG-INFIL_MAXSVS", 0x201100a2, "U1", 1, "",
         "Maximum number of satellites for navigation"),
        ("CFG-NAVSPG-INFIL_MINCNO", 0x201100a3, "U1", 1, "dBHz",
         "Minimum satellite signal level for navigation"),
        ("CFG-NAVSPG-INFIL_MINELEV", 0x201100a4, "I1", 1, "deg",
         "Minimum elevation for a GNSS satellite to be used in navigation"),
        ("CFG-NAVSPG-INFIL_NCNOTHRS", 0x201100aa, "U1", 1, "",
         "Number of satellites required to have C/N0 above "
         "CFG-NAVSPG-INFIL_CNOTHRS for a fix to be attempted"),
        ("CFG-NAVSPG-INFIL_CNOTHRS", 0x201100ab, "U1", 1, "",
         "C/N0 threshold for deciding whether to attempt a fix"),
        ("CFG-NAVSPG-OUTFIL_PDOP", 0x301100b1, "U2", 0.1, "",
         "Output filter position DOP mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_TDOP", 0x301100b2, "U2", 0.11, "",
         "Output filter time DOP mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_PACC", 0x301100b3, "U2", 1, "m",
         "Output filter position accuracy mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_TACC", 0x301100b4, "U2", 1, "m",
         "Output filter time accuracy mask (threshold)"),
        ("CFG-NAVSPG-OUTFIL_FACC", 0x301100b5, "U2", 0.01, "m/s",
         "Output filter frequency accuracy mask (threshold)"),
        ("CFG-NAVSPG-CONSTR_ALT", 0x401100c1, "I4", 0.01, "m",
         "Fixed altitude (mean sea level) for 2D fix mode"),
        ("CFG-NAVSPG-CONSTR_ALTVAR", 0x401100c2, "U4", 0.0001, "M^2",
         "Fixed altitude variance for 2D mode"),
        ("CFG-NAVSPG-CONSTR_DGNSSTO", 0x201100c4, "U1", 1, "s",
         "DGNSS timeout"),
        # CFG-NMEA-
        ("CFG-NMEA-PROTVER", 0x20930001, "E1", 1, "",
         "NMEA protocol version"),
        ("CFG-NMEA-MAXSVS", 0x20930002, "E1", 1, "",
         "Maximum number of SVs to report per Talker ID"),
        ("CFG-NMEA-COMPAT", 0x10930003, "L", 1, "",
         "Enable compatibility mode"),
        ("CFG-NMEA-CONSIDER", 0x10930004, "L", 1, "",
         "Enable considering mode"),
        ("CFG-NMEA-LIMIT82", 0x10930005, "L", 1, "",
         "Enable strict limit to 82 characters maximum NMEA message length"),
        ("CFG-NMEA-HIGHPREC", 0x10930006, "L", 1, "",
         "Enable high precision mode"),
        ("CFG-NMEA-SVNUMBERING", 0x20930007, "E1", 1, "",
         "Display configuration for SVs that have no value defined in NMEA"),
        ("CFG-NMEA-FILT_GPS", 0x10930011, "L", 1, "",
         "Disable reporting of GPS satellites"),
        ("CFG-NMEA-FILT_SBAS", 0x10930012, "L", 1, "",
         "Disable reporting of SBAS satellites"),
        ("CFG-NMEA-FILT_QZSS", 0x10930015, "L", 1, "",
         "Disable reporting of QZSS satellites"),
        ("CFG-NMEA-FILT_GLO", 0x10930016, "L", 1, "",
         "Disable reporting of GLONASS satellites"),
        ("CFG-NMEA-FILT_BDS", 0x10930017, "L", 1, "",
         "Disable reporting of BeiDou satellites"),
        ("CFG-NMEA-OUT_INVFIX", 0x10930021, "L", 1, "",
         "Enable position output for failed or invalid fixes"),
        ("CFG-NMEA-OUT_MSKFIX", 0x10930022, "L", 1, "",
         "Enable position output for invalid fixes"),
        ("CFG-NMEA-OUT_INVTIME", 0x10930023, "L", 1, "",
         "Enable time output for invalid times"),
        ("CFG-NMEA-OUT_INVDATE", 0x10930024, "L", 1, "",
         "Enable date output for invalid dates"),
        ("CFG-NMEA-OUT_ONLYGPS", 0x10930025, "L", 1, "",
         "Restrict output to GPS satellites only"),
        ("CFG-NMEA-OUT_FROZENCOG", 0x10930026, "L", 1, "",
         "Enable course over ground output even if it is frozen"),
        ("CFG-NMEA-MAINTALKERID", 0x20930031, "E1", 1, "",
         "Main Talker ID"),
        ("CFG-NMEA-GSVTALKERID", 0x20930032, "E1", 1, "",
         "Talker ID for GSV NMEA messages"),
        ("CFG-NMEA-BDSTALKERID", 0x30930033, "U2", 1, "",
         "BeiDou Talker ID"),
        # CFG-ODO-
        ("CFG-ODO-USE_ODO", 0x10220001, "L", 1, "",
         "Use odometer"),
        ("CFG-ODO-USE_COG", 0x10220002, "L", 1, "",
         "Use low-speed course over ground filter"),
        ("CFG-ODO-OUTLPVEL", 0x10220003, "L", 1, "",
         "Output low-pass filtered velocity"),
        ("CFG-ODO-OUTLPCOG", 0x10220004, "L", 1, "",
         "Output low-pass filtered course over ground (heading)"),
        ("CFG-ODO-PROFILE", 0x20220005, "E1", 1, "",
         "Odometer profile configuration"),
        ("CFG-ODO-COGMAXSPEED", 0x20220021, "U1", 1, "m/s",
         "Upper speed limit for low-speed course over ground filter"),
        ("CFG-ODO-COGMAXPOSACC", 0x20220022, "U1", 1, "",
         "Maximum acceptable position accuracy for computing low-speed  "
         "filtered course over ground"),
        ("CFG-ODO-COGLPGAIN", 0x20220032, "", 1, "",
         "Course over ground low-pass filter level (at speed < 8 m/s)"),
        ("CFG-ODO-VELLPGAIN", 0x20220031, "U1", 1, "",
         "Velocity low-pass filter level"),
        # CFG-RATE-
        ("CFG-RATE-MEAS", 0x30210001, "U2", 0.001, "s",
         "Nominal time between GNSS measurements"),
        ("CFG-RATE-NAV", 0x30210002, "U2", 1, "",
         "Ratio of number of measurements to number of navigation solutions"),
        ("CFG-RATE-TIMEREF", 0x20210003, "E1", 1, "",
         "Time system to which measurements are aligned"),
        # CFG-RINV-
        ("CFG-RINV-DUMP", 0x10c70001, "L", 1, "",
         "Dump data at startup"),
        ("CFG-RINV-BINARY", 0x10c70002, "L", 1, "",
         "Data is binary"),
        ("CFG-RINV-DATA_SIZE", 0x20c70003, "U1", 1, "",
         "Size of data"),
        ("CFG-RINV-CHUNK0", 0x50c70004, "X8", 1, "",
         "Data bytes 1-8 (LSB)"),
        ("CFG-RINV-CHUNK1", 0x50c70005, "X8", 1, "",
         "Data bytes 9-16"),
        ("CFG-RINV-CHUNK2", 0x50c70006, "X8", 1, "",
         "Data bytes 17-24"),
        ("CFG-RINV-CHUNK3", 0x50c70007, "X8", 1, "",
         "Data bytes 25-30 (MSB)"),
        # CFG-SBAS-
        ("CFG-SBAS-USE_TESTMODE", 0x10360002, "L", 1, "",
         "Use SBAS data when it is in test mode"),
        ("CFG-SBAS-USE_RANGING", 0x10360003, "L", 1, "",
         "Use SBAS GEOs as a ranging source (for navigation)"),
        ("CFG-SBAS-USE_DIFFCORR", 0x10360004, "L", 1, "",
         "Use SBAS differential corrections"),
        ("CFG-SBAS-USE_INTEGRITY", 0x10360005, "L", 1, "",
         "Use SBAS integrity information"),
        ("CFG-SBAS-PRNSCANMASK", 0x50360006, "X8", 1, "",
         "SBAS PRN search configuration"),
        # CFG-SIGNAL-
        ("CFG-SIGNAL-GPS_ENA", 0x1031001f, "L", 1, "",
         "GPS enable"),
        ("CFG-SIGNAL-GPS_L1CA_ENA", 0x10310001, "L", 1, "",
         "GPS L1C/A"),
        ("CFG-SIGNAL-GPS_L2C_ENA", 0x10310003, "L", 1, "",
         "GPS L2C"),
        ("CFG-SIGNAL-SBAS_ENA", 0x10310020, "L", 1, "",
         "SBAS enable"),
        ("CFG-SIGNAL-SBAS_L1CA_ENA", 0x10310005, "L", 1, "",
         "SBAS L1C/A"),
        ("CFG-SIGNAL-GAL_ENA", 0x10310021, "L", 1, "",
         "Galileo enable"),
        ("CFG-SIGNAL-GAL_E1_ENA", 0x10310007, "L", 1, "",
         "Galileo E1"),
        ("CFG-SIGNAL-GAL_E5B_ENA", 0x1031000a, "L", 1, "",
         "Galileo E5b"),
        ("CFG-SIGNAL-BDS_ENA", 0x10310022, "L", 1, "",
         "BeiDou Enable"),
        ("CFG-SIGNAL-BDS_B1_ENA", 0x1031000d, "L", 1, "",
         "BeiDou B1I"),
        ("CFG-SIGNAL-BDS_B2_ENA", 0x1031000e, "L", 1, "",
         "BeiDou B2I"),
        ("CFG-SIGNAL-QZSS_ENA", 0x10310024, "L", 1, "",
         "QZSS enable"),
        ("CFG-SIGNAL-QZSS_L1CA_ENA", 0x10310012, "L", 1, "",
         "QZSS L1C/A"),
        ("CFG-SIGNAL-QZSS_L1S_ENA", 0x10310014, "L", 1, "",
         "QZSS L1S"),
        ("CFG-SIGNAL-QZSS_L2C_ENA", 0x10310015, "L", 1, "",
         "QZSS L2C"),
        ("CFG-SIGNAL-GLO_ENA", 0x10310025, "L", 1, "",
         "GLONASS enable"),
        ("CFG-SIGNAL-GLO_L1_ENA", 0x10310018, "L", 1, "",
         "GLONASS L1"),
        ("CFG-SIGNAL-GLO_L2_ENA", 0x1031001a, "L", 1, "",
         "GLONASS L2"),
        # CFG-SPI-
        ("CFG-SPI-MAXFF", 0x20640001, "U1", 1, "",
         "Number of bytes containing 0xFF to receive before "
         "switching off reception."),
        ("CFG-SPI-CPOLARITY", 0x10640002, "L", 1, "",
         "Clock polarity select"),
        ("CFG-SPI-CPHASE", 0x10640003, "L", 1, "",
         "Clock phase select"),
        ("CFG-SPI-EXTENDEDTIMEOUT", 0x10640005, "L", 1, "",
         "Flag to disable timeouting the interface after 1.5s"),
        ("CFG-SPI-ENABLED", 0x10640006, "L", 1, "",
         "Flag to indicate if the SPI interface should be enabled"),
        # CFG-SPIINPROT-
        ("CFG-SPIINPROT-UBX", 0x10790001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on SPI"),
        ("CFG-SPIINPROT-NMEA", 0x10790002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on SPI"),
        ("CFG-SPIINPROT-RTCM2X", 0x10790003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on SPI"),
        ("CFG-SPIINPROT-RTCM3X", 0x10790004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on SPI"),
        # CFG-SPIOUTPROT-
        ("CFG-SPIOUTPROT-UBX", 0x107a0001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on SPI"),
        ("CFG-SPIOUTPROT-NMEA", 0x107a0002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on SPI"),
        ("CFG-SPIOUTPROT-RTCM3X", 0x107a0004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on SPI"),
        # CFG-TMODE-
        ("CFG-TMODE-MODE", 0x20030001, "E1", 1, "",
         "Receiver mode"),
        ("CFG-TMODE-POS_TYPE", 0x20030002, "E1", 1, "",
         "Determines whether the ARP position is given in ECEF or "
         "LAT/LON/HEIGHT?"),
        ("CFG-TMODE-ECEF_X", 0x40030003, "I4", 1, "cm",
         "ECEF X coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Y", 0x40030004, "I4", 1, "cm",
         "ECEF Y coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Z", 0x40030005, "I4", 1, "cm",
         "ECEF Z coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_X_HP", 0x20030006, "I1", 0.1, "mm",
         "High-precision ECEF X coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Y_HP", 0x20030007, "I1", 0.1, "mm",
         "High-precision ECEF Y coordinate of the ARP position."),
        ("CFG-TMODE-ECEF_Z_HP", 0x20030008, "I1", 0.1, "mm",
         "High-precision ECEF Z coordinate of the ARP position."),
        ("CFG-TMODE-LAT", 0x40030009, "I4", 1e-7, "deg",
         "Latitude of the ARP position."),
        ("CFG-TMODE-LON", 0x4003000a, "I4", 1e-7, "deg",
         "Longitude of the ARP position."),
        ("CFG-TMODE-HEIGHT", 0x4003000b, "I4", 1, "cm",
         "Height of the ARP position."),
        ("CFG-TMODE-LAT_HP", 0x2003000c, "I1", 1e-9, "deg",
         "High-precision latitude of the ARP position"),
        ("CFG-TMODE-LON_HP", 0x2003000d, "I1", 1e-9, "deg",
         "High-precision longitude of the ARP position."),
        ("CFG-TMODE-HEIGHT_HP", 0x2003000e, "I1", 0.1, "mm",
         "High-precision height of the ARP position."),
        ("CFG-TMODE-FIXED_POS_ACC", 0x4003000f, "U4", 0.1, "mm",
         "Fixed position 3D accuracy"),
        ("CFG-TMODE-SVIN_MIN_DUR", 0x40030010, "U4", 1, "s",
         "Survey-in minimum duration"),
        ("CFG-TMODE-SVIN_ACC_LIMIT", 0x40030011, "U4", 0.1, "mm",
         "Survey-in position accuracy limit"),
        # CFG-TP-
        ("CFG-TP-PULSE_DEF", 0x20050023, "E1", 1, "",
         "Determines whether the time pulse is interpreted as frequency "
         "or period?"),
        ("CFG-TP-PULSE_LENGTH_DEF", 0x20050030, "E1", 1, "",
         "Determines whether the time pulse length is interpreted as "
         "length[us] or pulse ratio[%]?"),
        ("CFG-TP-ANT_CABLEDELAY", 0x30050001, "I2", 0.000000001, "s",
         "Antenna cable delay"),
        ("CFG-TP-PERIOD_TP1", 0x40050002, "U4", 0.000001, "s",
         "Time pulse period (TP1)"),
        ("CFG-TP-PERIOD_LOCK_TP1", 0x40050003, "U4", 0.000001, "s",
         "Time pulse period when locked to GNSS time (TP1)"),
        ("CFG-TP-FREQ_TP1", 0x40050024, "U4", 1, "Hz",
         "Time pulse frequency (TP1)"),
        ("CFG-TP-FREQ_LOCK_TP1", 0x40050025, "U4", 1, "Hz",
         "Time pulse frequency when locked to GNSS time (TP1)"),
        ("CFG-TP-LEN_TP1", 0x40050004, "U4", 0.000001, "s",
         "Time pulse length (TP1)"),
        ("CFG-TP-LEN_LOCK_TP1", 0x40050005, "U4", 0.000001, "s",
         "Time pulse length when locked to GNSS time (TP1)"),
        ("CFG-TP-DUTY_TP1", 0x5005002a, "R8", 1, "%",
         "Time pulse duty cycle (TP1)"),
        ("CFG-TP-DUTY_LOCK_TP1", 0x5005002b, "R8", 1, "%",
         "Time pulse duty cycle when locked to GNSS time (TP1)"),
        ("CFG-TP-USER_DELAY_TP1", 0x40050006, "I4", 0.000000001, "s",
         "User configurable time pulse delay (TP1)"),
        ("CFG-TP-TP1_ENA", 0x10050007, "L", 1, "",
         "Enable the first timepulse"),
        ("CFG-TP-SYNC_GNSS_TP1", 0x10050008, "L", 1, "",
         "Sync time pulse to GNSS time or local clock (TP1)"),
        ("CFG-TP-USE_LOCKED_TP1", 0x10050009, "L", 1, "",
         "Use locked parameters when possible (TP1)"),
        ("CFG-TP-ALIGN_TO_TOW_TP1", 0x1005000a, "L", 1, "",
         "Align time pulse to top of second (TP1)"),
        ("CFG-TP-POL_TP1", 0x1005000b, "L", 1, "",
         "Set time pulse polarity (TP1)"),
        ("CFG-TP-TIMEGRID_TP1", 0x2005000c, "E1", 1, "",
         "Time grid to use (TP1)"),
        ("CFG-TP-PERIOD_TP2", 0x4005000d, "U4", 0.000001, "s",
         "Time pulse period (TP2)"),
        ("CFG-TP-PERIOD_LOCK_TP2", 0x4005000e, "U4", 0.000001, "s",
         "Time pulse period when locked to GNSS time (TP2)"),
        ("CFG-TP-FREQ_TP2", 0x40050026, "U4", 1, "Hz",
         "Time pulse frequency (TP2)"),
        ("CFG-TP-FREQ_LOCK_TP2", 0x40050027, "U4", 1, "Hz",
         "Time pulse frequency when locked to GNSS time (TP2)"),
        ("CFG-TP-LEN_TP2", 0x4005000f, "U4", 0.000001, "s",
         "Time pulse length (TP2)"),
        ("CFG-TP-LEN_LOCK_TP2", 0x40050010, "U4", 0.000001, "s",
         "Time pulse length when locked to GNSS time (TP2)"),
        ("CFG-TP-DUTY_TP2", 0x5005002c, "R8", 1, "%",
         "Time pulse duty cycle (TP2)"),
        ("CFG-TP-DUTY_LOCK_TP2", 0x5005002d, "R8", 1, "%",
         "Time pulse duty cycle when locked to GNSS time (TP2)"),
        ("CFG-TP-USER_DELAY_TP2", 0x40050011, "I4", 0.000000001, "s",
         "User configurable time pulse delay (TP2)"),
        ("CFG-TP-TP2_ENA", 0x10050012, "L", 1, "",
         "Enable the second timepulse"),
        ("CFG-TP-SYNC_GNSS_TP2", 0x10050013, "L", 1, "",
         "Sync time pulse to GNSS time or local clock (TP2)"),
        ("CFG-TP-USE_LOCKED_TP2", 0x10050014, "L", 1, "",
         "Use locked parameters when possible (TP2)"),
        ("CFG-TP-ALIGN_TO_TOW_TP2", 0x10050015, "L", 1, "",
         "Align time pulse to top of second (TP2)"),
        ("CFG-TP-POL_TP2", 0x10050016, "L", 1, "",
         "Set time pulse polarity (TP2)"),
        ("CFG-TP-TIMEGRID_TP2", 0x20050017, "E1", 1, "",
         "Time grid to use (TP2)"),
        # CFG-UART1-
        ("CFG-UART1-BAUDRATE", 0x40520001, "U4", 1, "",
         "The baud rate that should be configured on the UART1"),
        ("CFG-UART1-STOPBITS", 0x20520002, "E1", 1, "",
         "Number of stopbits that should be used on UART1"),
        ("CFG-UART1-DATABITS", 0x20520003, "E1", 1, "",
         "Number of databits that should be used on UART1"),
        ("CFG-UART1-PARITY", 0x20520004, "E1", 1, "",
         "Parity mode that should be used on UART1"),
        ("CFG-UART1-ENABLED", 0x10520005, "L", 1, "",
         "Flag to indicate if the UART1 should be enabled"),
        # CFG-UART1INPROT
        ("CFG-UART1INPROT-UBX", 0x10730001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on UART1"),
        ("CFG-UART1INPROT-NMEA", 0x10730002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on UART1"),
        ("CFG-UART1INPROT-RTCM2X", 0x10730003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on UART1"),
        ("CFG-UART1INPROT-RTCM3X", 0x10730004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on UART1"),
        # CFG-UART1OUTPROT
        ("CFG-UART1OUTPROT-UBX", 0x10740001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on UART1"),
        ("CFG-UART1OUTPROT-NMEA", 0x10740002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on UART1"),
        ("CFG-UART1OUTPROT-RTCM3X", 0x10740004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on UART1"),
        # CFG-UART2-
        ("CFG-UART2-BAUDRATE", 0x40530001, "U4", 1, "",
         "The baud rate that should be configured on the UART2"),
        ("CFG-UART2-STOPBITS", 0x20530002, "E1", 1, "",
         "Number of stopbits that should be used on UART2"),
        ("CFG-UART2-DATABITS", 0x20530003, "E1", "1", "",
         "Number of databits that should be used on UART2"),
        ("CFG-UART2-PARITY", 0x20530004, "E1", "1", "",
         "Parity mode that should be used on UART2"),
        ("CFG-UART2-ENABLED", 0x10530005, "L", "1", "",
         "Flag to indicate if the UART2 should be enabled"),
        ("CFG-UART2-REMAP", 0x10530006, "L", "1", "",
         "UART2 Remapping"),
        # CFG-UART1INPROT
        ("CFG-UART2INPROT-UBX", 0x10750001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on UART2"),
        ("CFG-UART2INPROT-NMEA", 0x10750002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on UART2"),
        ("CFG-UART2INPROT-RTCM2X", 0x10750003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on UART2"),
        ("CFG-UART2INPROT-RTCM3X", 0x10750004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on UART2"),
        # CFG-UART1OUTPROT
        ("CFG-UART2OUTPROT-UBX", 0x10760001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on UART2"),
        ("CFG-UART2OUTPROT-NMEA", 0x10760002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on UART2"),
        ("CFG-UART2OUTPROT-RTCM3X", 0x10760004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on UART2"),
        # CFG-USB-
        ("CFG-USB-ENABLED", 0x10650001, "L", 1, "",
         "Flag to indicate if the USB interface should be enabled"),
        ("CFG-USB-SELFPOW", 0x10650002, "L", 1, "",
         "Self-Powered device"),
        ("CFG-USB-VENDOR_ID", 0x3065000a, "U2", 1, "",
         "Vendor ID"),
        ("CFG-USB-PRODUCT_ID", 0x3065000b, "U2", 1, "",
         "Product ID"),
        ("CFG-USB-POWER", 0x3065000c, "U2", 1, "mA",
         "Power consumption"),
        ("CFG-USB-VENDOR_STR0", 0x5065000d, "X8", 1, "",
         "Vendor string characters 0-7"),
        ("CFG-USB-VENDOR_STR1", 0x5065000e, "X8", 1, "",
         "Vendor string characters 8-15"),
        ("CFG-USB-VENDOR_STR2", 0x5065000f, "X8", 1, "",
         "Vendor string characters 16-23"),
        ("CFG-USB-VENDOR_STR3", 0x50650010, "X8", 1, "",
         "Vendor string characters 24-31"),
        ("CFG-USB-PRODUCT_STR0", 0x50650011, "X8", 1, "",
         "Product string characters 0-7"),
        ("CFG-USB-PRODUCT_STR1", 0x50650012, "X8", 1, "",
         "Product string characters 8-15"),
        ("CFG-USB-PRODUCT_STR2", 0x50650013, "X8", 1, "",
         "Product string characters 16-23"),
        ("CFG-USB-PRODUCT_STR3", 0x50650014, "X8", 1, "",
         "Product string characters 24-31"),
        ("CFG-USB-SERIAL_NO_STR0", 0x50650015, "X8", 1, "",
         "Serial number string characters 0-7"),
        ("CFG-USB-SERIAL_NO_STR1", 0x50650016, "X8", 1, "",
         "Serial number string characters 8-15"),
        ("CFG-USB-SERIAL_NO_STR2", 0x50650017, "X8", 1, "",
         "Serial number string characters 16-23"),
        ("CFG-USB-SERIAL_NO_STR3", 0x50650018, "X8", 1, "",
         "Serial number string characters 24-31"),
        # CFG-USB-INPROT
        ("CFG-USBINPROT-UBX", 0x10770001, "L", 1, "",
         "Flag to indicate if UBX should be an input protocol on USB"),
        ("CFG-USBINPROT-NMEA", 0x10770002, "L", 1, "",
         "Flag to indicate if NMEA should be an input protocol on USB"),
        ("CFG-USBINPROT-RTCM2X", 0x10770003, "L", 1, "",
         "Flag to indicate if RTCM2X should be an input protocol on USB"),
        ("CFG-USBINPROT-RTCM3X", 0x10770004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an input protocol on USB"),
        # CFG-USB-OUTPROT
        ("CFG-USBOUTPROT-UBX", 0x10780001, "L", 1, "",
         "Flag to indicate if UBX should be an output protocol on USB"),
        ("CFG-USBOUTPROT-NMEA", 0x10780002, "L", 1, "",
         "Flag to indicate if NMEA should be an output protocol on USB"),
        ("CFG-USBOUTPROT-RTCM3X", 0x10780004, "L", 1, "",
         "Flag to indicate if RTCM3X should be an output protocol on USB"),
       )

    def item_to_type(self, item):
        """Return (size, pack format, i/i/f) for item"""

        # conversion of known types from known key
        cfg_types = {
                     "E1": (1, "<B", "u"),
                     "E2": (2, "<H", "u"),
                     "E4": (4, "<L", "u"),
                     "I1": (1, "<b", "i"),
                     "I2": (2, "<h", "i"),
                     "I4": (4, "<l", "i"),
                     "I8": (8, "<q", "i"),
                     "L": (1, "<B", "u"),
                     "R4": (4, "<f", "f"),
                     "R8": (2, "<d", "f"),
                     "U1": (1, "<B", "u"),
                     "U2": (2, "<H", "u"),
                     "U4": (4, "<L", "u"),
                     "U8": (8, "<Q", "u"),
                     "X1": (1, "<B", "u"),
                     "X2": (2, "<H", "u"),
                     "X4": (4, "<L", "u"),
                     "X8": (8, "<Q", "u"),
                     }
        # guess of known types from unknown key
        key_map = {0: (1, "<B", "u"),       # illegal
                   1: (1, "<B", "u"),       # one bit
                   2: (1, "<B", "u"),       # one byte
                   3: (2, "<H", "u"),       # two byte
                   4: (4, "<L", "u"),       # four byte
                   5: (8, "<B", "u"),       # eight byte
                   6: (1, "<B", "u"),       # illegal
                   7: (1, "<B", "u"),       # illegal
                   }

        key = item[1]
        val_type = item[2]
        if val_type in cfg_types:
            cfg_type = cfg_types[val_type]
        else:
            # unknown? get length correct
            key_size = (key >> 28) & 0x07
            cfg_type = key_map[key_size]

        return cfg_type

    def cfg_by_key(self, key):
        """Find a config item by key"""

        for item in self.cfgs:
            if item[1] == key:
                return item

        # not found, build a fake item, guess on decode
        name = "CFG-%u-%u" % ((key >> 16) & 0xff, ket & 0xff)
        map = {0: "Z0",
               1: "L",
               2: "U1",
               3: "U2",
               4: "U4",
               5: "U8",
               6: "Z6",
               7: "Z7",
               }
        size = (key >> 28) & 0x07
        item = (name, key, map[size], 1, "Unk", "Unknown")

        return item

    def cfg_by_name(self, name):
        """Find a config item by name"""

        for item in self.cfgs:
            if item[0] == name:
                return item

        return None

    id_map = {
        0: {"name": "GPS",
            "sig": {0: "L1C/A", 3: "L2 CL", 4: "L2 CM"}},
        1: {"name": "SBAS",
            "sig": {0: "L1C/A", 3: "L2 CL", 4: "L2 CM"}},
        2: {"name": "Galileo",
            "sig": {0: "E1C", 1: "E1 B", 5: "E5 bl", 6: "E5 bQ"}},
        3: {"name": "BeiDou",
            "sig": {0: "B1I D1", 1: "B1I D2", 2: "B2I D1", 3: "B2I D2"}},
        4: {"name": "IMES",
            "sig": {0: "L1C/A", 3: "L2 CL", 4: "L2 CM"}},
        5: {"name": "QZSS",
            "sig": {0: "L1C/A", 4: "L2 CM", 5: "L2 CL"}},
        6: {"name": "GLONASS",
            "sig": {0: "L1 OF", 2: "L2 OF"}},
    }

    def gnss_s(self, gnssId, svId, sigId):
        """Verbose decode of gnssId, svId and sigId"""

        s = ''

        if gnssId in self.id_map:
            if "name" not in self.id_map[gnssId]:
                s = "%d PRN %d sigId %d" % (gnssId, svId, sigId)
            elif sigId not in self.id_map[gnssId]["sig"]:
                s = ("%s PRN %d sigId %d" %
                     (self.id_map[gnssId]["name"], svId, sigId))
            else:
                s = ("%s PRN %d sigId %s" %
                     (self.id_map[gnssId]["name"], svId,
                      self.id_map[gnssId]["sig"][sigId]))
        else:
            s = "%d PRN %d sigId %d" % (gnssId, svId, sigId)

        return s

    def ack_ack(self, buf):
        """UBX-ACK-ACK decode"""
        m_len = len(buf)
        if 2 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BB', buf, 0)
        return ' ACK to: %s' % self.class_id_s(u[0], u[1])

    ack_ids = {0: {'str': 'NAK', 'dec': ack_ack, 'name': 'UBX-ACK-NAK'},
               1: {'str': 'ACK', 'dec': ack_ack, 'name': 'UBX-ACK-ACK'}}

    cfg_ant_pins = {
        1: 'svcs',
        2: 'scd',
        4: 'ocd',
        8: 'pdwnOnSCD',
        0x10: 'recovery',
        }

    def cfg_ant(self, buf):
        """UBX-CFG-ANT decode"""
        m_len = len(buf)
        if 0 == m_len:
            return "Poll request all"

        if 4 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<HH', buf, 0)
        s = ' flags x%x pins x%x ' % u
        s += ('pinSwitch %u pinSCD %u pinOCD %u reconfig %u' %
              (u[1] & 0x1f, (u[1] >> 5) & 0x1f, (u[1] >> 10) & 0x1f,
               u[1] >> 15))
        if VERB_DECODE <= opts['verbosity']:
            s += ('\n      flags (%s)' % flag_s(u[0], self.cfg_ant_pins))

        return s

    cfg_cfg_mask = {
        0x1: 'ioPort',
        0x2: 'msgConf',
        0x4: 'infMsg',
        0x8: 'navConf',
        0x10: 'rxmConf',
        0x100: 'senConf',        # not on M8030
        0x200: 'rinvConf',
        0x400: 'antConf',
        0x800: 'logConf',
        0x1000: 'ftsConf',
        }

    cfg_cfg_dev = {
        0x1: 'devBBR',
        0x2: 'devFlash',
        0x4: 'devEEPROM',
        0x10: 'devSpiFlash',
        }

    def cfg_cfg(self, buf):
        """UBX-CFG-CFG decode"""
        m_len = len(buf)
        if 12 > m_len:
            return "Bad Length %s" % m_len

        if 12 == m_len:
            u = struct.unpack_from('<LLL', buf, 0)
        else:
            u = struct.unpack_from('<LLLB', buf, 0)

        s = (' clearMask: %#x (%s)\n' %
             (u[0], flag_s(u[0], self.cfg_cfg_mask)))
        s += (' saveMask: %#x (%s)\n' %
              (u[1], flag_s(u[1], self.cfg_cfg_mask)))
        s += (' loadMask: %#x (%s)\n' %
              (u[2], flag_s(u[2], self.cfg_cfg_mask)))

        if 13 <= m_len:
            s += (' deviceMask: %#x (%s)\n' %
                  (u[3], flag_s(u[3], self.cfg_cfg_dev)))

        return s

    cfg_dgnss_mode = {
        2: "RTK Float",
        3: "RTK Fixed",
        }

    def cfg_dgnss(self, buf):
        """UBX-CFG-DGNSS decode, DGNSS configuration"""
        m_len = len(buf)
        if 0 == m_len:
            return "Poll request"

        if 4 > m_len:
            return "Bad Length %d" % m_len

        u = struct.unpack_from('<BBBB', buf, 0)
        s = (" dgnssMode %u (%s) reserved1 %u %u %u" % u
             (u[0], index_s(u[0], self.cfg_dgnss_mode), u[1], u[2], u[3]))
        return s

    # signals defined in protver 27+
    # signals used in protver 15+
    # top byte used, but not defined
    cfg_gnss_sig = {
        0: {0x010000: "L1C/A",    # GPS
            0x100000: "L2C"},
        1: {0x010000: "L1C/A"},   # SBAS
        2: {0x010000: "E1",       # Galileo
            0x200000: "E5b"},
        3: {0x010000: "B1I",      # BeiDou
            0x100000: "B2I"},
        4: {0x010000: "L1"},      # IMES
        5: {0x010000: "L1C/A",    # QZSS
            0x040000: "L2S",
            0x100000: "L2C"},
        6: {0x010000: "L1",       # GLONASS
            0x100000: "L2"},
        }

    def cfg_gnss(self, buf):
        """UBX-CFG-GNSS decode, GNSS system configuration"""
        m_len = len(buf)
        if 0 == m_len:
            return "Poll request"

        if 4 > m_len:
            return "Bad Length %d" % m_len

        u = struct.unpack_from('<BBBB', buf, 0)
        s = " msgVer %u  numTrkChHw %u numTrkChUse %u numConfigBlocks %u" % u

        for i in range(0, u[3]):
            u = struct.unpack_from('<BBBBL', buf, 4 + (i * 8))
            sat = u[0]
            s += ("\n  gnssId %u TrkCh %2u maxTrCh %2u reserved %u "
                  "Flags x%08x\n" % u)

            if 7 > sat:
                s += ("   %s %s " %
                      (index_s(sat, self.gnss_id),
                       flag_s(u[4], self.cfg_gnss_sig[sat])))
            else:
                s += "Unk "

            if u[4] & 0x01:
                s += 'enabled'

        return s

    cfg_inf_protid = {
        0: "UBX",
        1: "NMEA",
        }

    def cfg_inf(self, buf):
        """UBX-CFG-INF decode, Poll configuration for one protocol"""
        m_len = len(buf)
        if 1 == m_len:
            return ("Poll request: %s" %
                    index_s(buf[0], self.cfg_inf_protid))

        if 10 < m_len:
            return "Bad Length %d" % m_len

        u = struct.unpack_from('<BBBBBBBBBB', buf, 0)
        s = (" protocolId %u reserved1 %u %u %u\n"
             " infMsgMask %u %u %u %u %u %u" % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ('\n   protocolId (%s)' %
                  index_s(buf[0], self.cfg_inf_protid))
        return s

    utc_std = {
            0: "Default",
            1: "CRL",
            2: "NIST",
            3: "USNO",
            4: "BIPM",
            5: "tbd",
            6: "SU",
            7: "NTSC",
        }

    cfg_nav5_dyn = {
        0: "Portable",
        2: "Stationary",
        3: "Pedestrian",
        4: "Automotive",
        5: "Sea",
        6: "Airborne with <1g Acceleration",
        7: "Airborne with <2g Acceleration",
        8: "Airborne with <4g Acceleration",
        }

    cfg_nav5_fix = {
        1: "2D only",
        2: "3D only",
        3: "Auto 2D/3D",
        }

    cfg_nav5_mask = {
        1: "dyn",
        2: "minEl",
        4: "posFixMode",
        8: "drLim",
        0x10: "posMask",
        0x20: "timeMask",
        0x40: "staticHoldMask",
        0x80: "dgpsMask",
        0x100: "cnoThreshold",
        0x400: "utc",
        }

    def cfg_nav5(self, buf):
        """UBX-CFG-NAV5 nav Engine Settings"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 36 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<HBBlLbBHHHHbbbbHHbBL', buf, 0)
        s = (' mask %#x dynModel %u fixmode %d fixedAlt %d FixedAltVar %u\n'
             ' minElev %d drLimit %u pDop %u tDop %u pAcc %u tAcc %u\n'
             ' staticHoldThresh %u dgpsTimeOut %u cnoThreshNumSVs %u\n'
             ' cnoThresh %u res %u staticHoldMaxDist %u utcStandard %u\n'
             ' reserved x%x %x' % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   dynModel (%s) fixMode (%s) utcStandard (%s)"
                  "\n   mask (%s)" %
                  (index_s(u[1], self.cfg_nav5_dyn),
                   index_s(u[2], self.cfg_nav5_fix),
                   index_s(u[17] >> 4, self.utc_std),
                   flag_s(u[0] >> 4, self.cfg_nav5_mask)))
        return s

    cfg_navx5_mask1 = {
        4: "minMax",
        8: "minCno",
        0x40: "initial3dfix",
        0x200: "wknRoll",
        0x400: "ackAid",
        0x2000: "ppp",
        0x4000: "aop",
        }

    cfg_navx5_mask2 = {
        0x40: "adr",
        0x80: "sigAttenComp",
        }

    cfg_navx5_aop = {
        1: "useAOP",
        }

    def cfg_navx5(self, buf):
        """UBX-CFG-NAVX5 decode, Navigation Engine Expert Settings"""

        # deprecated protver 23+
        # length == 20 case seems broken?
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<HHLHBBBBBHBH', buf, 0)
        s = (" version %u mask1 x%x mask2 x%x reserved1 %u minSVs %u "
             "maxSVs %u minCNO %u\n"
             " reserved2 %u iniFix3D %u reserved3 %u  ackAiding %u wknRollover %u" % u)

        # length == 40 in protver 27
        if 40 <= m_len:
            u1 = struct.unpack_from('<BBHHLHBB', buf, 20)
            s += ("\n sigAttenCompMode %u reserved456 %u %u %u usePPP %u "
                  "aopCfg %u reserved7 %u"
                  "\n aopOrbMaxErr %u reserved89 %u %u %u useAdr %u" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   mask1 (%s)"
                  "\n   mask2 (%s) aopCfg (%s)" %
                  (flag_s(u[1], self.cfg_navx5_mask1),
                   flag_s(u[2], self.cfg_navx5_mask2),
                   flag_s(u1[5], self.cfg_navx5_aop)))

        return s

    def cfg_msg(self, buf):
        """UBX-CFG-MSG decode"""
        m_len = len(buf)
        if 2 == m_len:
            u = struct.unpack_from('<BB', buf, 0)
            return ' Rate request: %s' % self.class_id_s(u[0], u[1])

        if 3 == m_len:
            u = struct.unpack_from('<BBB', buf, 0)
            return (' Rate set: %s Rate:%d' %
                    (self.class_id_s(u[0], u[1]), u[2]))

        if 8 != m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = (' %s Rates: %u %u %u %u %u %u' %
             (self.class_id_s(u[0], u[1]), u[2], u[3], u[4], u[5], u[6], u[7]))
        return s

    cfg_nmea_filter = {
        1: "posFilt",
        2: "mskPosFilt",
        4: "timeFilt",
        8: "dateFilt",
        0x10: "gpsOnlyFilter",
        0x20: "trackFilt",
        }

    cfg_nmea_ver = {
        0x11: "2.1",
        0x23: "2,3",
        0x40: "4.0",
        0x41: "4.10",
        }

    cfg_nmea_flags = {
        1: "compat",
        2: "consider",
        4: "limit82",
        8: "highPrec",
        }

    cfg_nmea_svn = {
        0: "Strict",
        1: "Extended",
        }

    cfg_nmea_mtid = {
        0: "Default",
        1: "GP",
        2: "GL",
        3: "GN",
        4: "GA",
        5: "GB",
        }

    cfg_nmea_gtid = {
        0: "GNSS Specific",
        1: "Main",
        }

    cfg_nmea_gnssfilt = {
        1: "gps",
        2: "sbas",
        0x10: "qzss",
        0x20: "glonass",
        0x40: "beidou",
        }

    def cfg_nmea(self, buf):
        """UBX-CFG-NMEA decode, Extended NMEA protocol configuration V1"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLBBBBBBBBBBBB', buf, 0)
        s = (" filter x%x nmeaVersion x%x numSv %u flags x%x "
             "gnssToFilter x%x\n"
             " svNumbering %u mainTalkerId %u gsvTalkerId %u version %u\n"
             " bdsTalkerId %u %u reserved1 %u %u %u %u %u %u" % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n  filter (%s) NMEA Version (%s) numSv (%s) flags (%s)"
                  "\n  gnssToFilter (%s) svNumbering (%s) mainTalkerId (%s)"
                  "\n  gsvTalkerId (%s)" %
                  (flag_s(u[0], self.cfg_nmea_filter),
                   index_s(u[1], self.cfg_nmea_ver),
                   u[2] if 0 != u[2] else "Unlimited",
                   flag_s(u[3], self.cfg_nmea_flags),
                   flag_s(u[4], self.cfg_nmea_gnssfilt),
                   index_s(u[5], self.cfg_nmea_svn),
                   index_s(u[6], self.cfg_nmea_mtid),
                   index_s(u[7], self.cfg_nmea_gtid)))
        return s

    cfg_odo_flags = {
        1: "useODO",
        2: "useCOG",
        4: "useLPVel",
        8: "useLPCog",
        }

    cfg_odo_profile = {
        0: "Running",
        1: "Cycling",
        2: "Swimming",
        3: "Car",
        4: "Custom",
        }

    def cfg_odo(self, buf):
        """UBX-CFG-ODO decode, Odometer, Low-speed COG Engine Settings"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBBBBBBBBBBBBBB', buf, 0)
        s = (" version %u reserved1 %u %u %u flags x%x odoCfg x%x\n"
             " reserved2 %u %u %u %u %u %u\n"
             " cagMaxSpeed %u cogMaxPosAcc %u reserved3 %u %u\n"
             " velLpGain %u cogLpGain %u reserved4 %u %u" % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   flags (%s) odoCfg (%s)" %
                  (flag_s(u[4], self.cfg_odo_flags),
                   index_s(u[5], self.cfg_odo_profile)))
        return s

    cfg_pms_values = {0: "Full power",
                      1: "Balanced",
                      2: "Interval",
                      3: "Aggresive with 1Hz",
                      4: "Aggresive with 2Hz",
                      5: "Aggresive with 4Hz",
                      0xff: "Invalid"
                      }

    def cfg_pms(self, buf):
        """UBX-CFG-PMS decode, Power Mode Setup"""

        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBHHBB', buf, 0)
        s = (' version %u powerSetupValue %u'
             ' period %u onTime %#x reserved1 %u %u' % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ('\n  powerSetupValue (%s)' %
                  index_s(u[0], self.cfg_pms_values))

        return s

    cfg_prt_flags = {
        0x2: 'extendedTxTimeout',
        }

    cfg_prt_proto = {
        0x1: 'UBX',
        0x2: 'NMEA',
        0x4: 'RTCM2',
        0x20: 'RTCM3',
        }

    def cfg_prt(self, buf):
        """UBX-CFG-PRT decode"""

        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        portid = buf[0]
        idstr = '%u (%s)' % (portid, self.port_ids.get(portid, '?'))

        if 1 == m_len:
            return " Poll request PortID %s" % idstr

        # Note that this message can contain multiple 20-byte submessages, but
        # only in the send direction, which we don't currently do.
        if 20 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBHLLHHHH', buf, 0)

        s = [' PortID %s reserved1 %u txReady %#x' % (idstr, u[1], u[2])]
        s.append({1: '  mode %#x baudRate %u',
                  2: '  mode %#x baudRate %u',
                  3: '  reserved2 [%u %u]',
                  4: '  mode %#x reserved2 %u',
                  0: '  mode %#x reserved2 %u',
                  }.get(portid, '  ???: %u,%u') % tuple(u[3:5]))
        s.append('  inProtoMask %#x outProtoMask %#x' % tuple(u[5:7]))
        s.append({1: '  flags %#x reserved2 %u',
                  2: '  flags %#x reserved2 %u',
                  3: '  reserved3 %u reserved4 %u',
                  4: '  flags %#x reserved3 %u',
                  0: '  flags %#x reserved3 %u',
                  }.get(portid, '  ??? %u,%u') % tuple(u[7:]))

        if portid == 0:
            s.append('    slaveAddr %#x' % (u[3] >> 1 & 0x7F))

        s.append('    inProtoMask (%s)\n'
                 '    outProtoMask (%s)' %
                 (flag_s(u[5], self.cfg_prt_proto),
                  flag_s(u[6], self.cfg_prt_proto)))

        if portid in set([1, 2, 4, 0]):
            s.append('    flags (%s)' % flag_s(u[7], self.cfg_prt_flags))

        return '\n'.join(s)

    cfg_rate_system = {
        0: "UTC",
        1: "GPS",
        2: "GLONASS",
        3: "BeiDou",
        4: "Galileo",
        }

    def cfg_rate(self, buf):
        """UBX-CFG-RATE decode, Navigation/Measurement Rate Settings"""

        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 6 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<HHH', buf, 0)
        s = (" measRate %u navRate %u timeRef %u (%s)" %
             (u + (index_s(u[2], self.cfg_rate_system),)))
        return s

    cfg_rinv_flags = {
        1: "dump",
        2: "binary",
        }

    def cfg_rinv(self, buf):
        """UBX-CFG-RINV decode, Contents of Remote Inventory"""

        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        u = struct.unpack_from('<B', buf, 0)
        s = (" flags x%x (%s) data:" %
             (u + (flag_s(u[0], self.cfg_rinv_flags),)))
        for i in range(0, m_len - 1):
            if 0 == (i % 8):
                s += "\n   "
            u = struct.unpack_from('<B', buf, i + 1)
            s += " %3u" % u

        return s

    def cfg_rst(self, buf):
        """"UBX-CFG-RST decode, Reset Receiver"""

        m_len = len(buf)

        if 4 > m_len:
            return "Bad Length %s" % m_len

        bbrvalues = {0: "Hot Start",
                     1: "Warm Start",
                     0xffff: "Cold Start",
                     }
        mode = {0: "Hardware reset",
                1: "Software reset",
                2: "Software reset (GNSS only)",
                4: "Hardware reset, after shutdown",
                8: "Controled GNSS stop",
                9: "Controled GNSS start",
                }
        u = struct.unpack_from('<HBB', buf, 0)
        s = ' navBbrmask: %#x resetMode: %u reserved: %u\n' % u
        if u[0] in bbrvalues:
            s += "   %s, " % bbrvalues[u[0]]
        else:
            s += "   Other, "
        if u[1] in mode:
            s += mode[u[1]]
        else:
            s += "Other"

        return s

    cfg_sbas_mode = {
        1: "enabled",
        2: "test",
        }

    cfg_sbas_usage = {
        1: "range",
        2: "diffCorr",
        3: "integrity",
        }

    cfg_sbas_scanmode1 = {
        1: "PRN120",
        2: "PRN121",
        4: "PRN122",
        8: "PRN123",
        0x10: "PRN124",
        0x20: "PRN125",
        0x40: "PRN126",
        0x80: "PRN127",
        0x100: "PRN128",
        0x200: "PRN129",
        0x400: "PRN130",
        0x800: "PRN131",
        0x1000: "PRN132",
        0x2000: "PRN133",
        0x4000: "PRN134",
        0x8000: "PRN135",
        0x10000: "PRN136",
        0x20000: "PRN137",
        0x40000: "PRN138",
        0x80000: "PRN139",
        0x100000: "PRN140",
        0x200000: "PRN141",
        0x400000: "PRN142",
        0x800000: "PRN143",
        0x1000000: "PRN144",
        0x2000000: "PRN145",
        0x4000000: "PRN146",
        0x8000000: "PRN147",
        0x10000000: "PRN148",
        0x20000000: "PRN149",
        0x40000000: "PRN150",
        0x80000000: "PRN151",
        }

    cfg_sbas_scanmode2 = {
        1: "PRN152",
        2: "PRN153",
        4: "PRN154",
        8: "PRN155",
        0x10: "PRN156",
        0x20: "PRN157",
        0x40: "PRN158",
        }

    def cfg_sbas(self, buf):
        """UBX-CFG-SBAS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBL', buf, 0)
        s = (" mode x%x usage x%x maxSBAS %u scanMode2 x%x"
             " scanMode1: x%x" % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   mode (%s) usage (%s) scanmode2 (%s)"
                  "\n   scanmode1 (%s)" %
                  (flag_s(u[0], self.cfg_sbas_mode),
                   flag_s(u[1], self.cfg_sbas_usage),
                   flag_s(u[3], self.cfg_sbas_scanmode2),
                   flag_s(u[4], self.cfg_sbas_scanmode1)))

        return s

    def cfg_smgr(self, buf):
        """UBX-CFG-SMGR decode, Synchronization manager configuration"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBHHBBHHHHL', buf, 0)
        s = (" version %u minGNSSFix %u maxFreqChangeRate %u "
             "maxPhaseCorrR %u\n"
             " reserved1 %u %u freqTolerance %u timeTolerance %u "
             "messageCfg x%x\n"
             " maxSlewRate %u flags x%x" % u)
        return s

    def cfg_tmode2(self, buf):
        """UBX-CFG-TMODE2 decode, Time Mode Settings 2"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBHlllLLL', buf, 0)
        s = (' timeMode: %u reserved1: %u  usage: %#x\n'
             '  ecefXOrLat: %d ecefYOrLon: %d ecefZOrLon: %d\n'
             '  fixeedPosAcc %u svinMinDur %u svinAccLimit %u' % u)
        return s

    cfg_tp5_flags = {
        1: "Active",
        2: "lockGnssFreq",
        4: "lockedOtherSet",
        8: "isFreq",
        0x10: "isLength",
        0x20: "alignToTow",
        0x40: "RisingEdge",
        }

    cfg_tp5_grid = {
        0: 'UTC',
        1: 'GPS',
        2: 'Glonass',
        3: 'BeiDou',
        4: 'Galileo',
        }

    def cfg_tp5(self, buf):
        """UBX-CFG-TP5 decode, Time Pulse Parameters"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request tpIdx 0"

        if 1 == m_len:
            return " Poll request tpIdx %d" % buf[0]

        if 32 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBHhhLLLLlL', buf, 0)
        s = ("tpIdx %u version %u reserved1 %u\n"
             " antCableDelay %d rfGroupDelay %d freqPeriod %u "
             "freqPeriodLock %u\n"
             " pulseLenRatio %u pulseLenRatioLock %u userConfigDelay %d\n"
             " Flags %#x" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   Flags (%s)"
                  "\n   gridToGps (%s) syncMode %d" %
                  (flag_s(u[10] & 0x7f, self.cfg_tp5_flags),
                   index_s((u[10] >> 7) & 0x0f, self.cfg_tp5_grid),
                   (u[10] >> 11) & 0x03))

        return s

    def cfg_usb(self, buf):
        """UBX-CFG-USB decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 108 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<HHHHHH', buf, 0)
        s = (' vendorID: %#x productID: %#x reserved1: %u'
             ' reserved2: %u\n'
             '  powerConsumption %u mA flags: %#x ' % tuple(u[:6]))
        if 0x01 & u[5]:
            s += "reEnum, "
        if 0x02 & u[5]:
            s += "self-powered"
        else:
            s += "bus-powered"

        s += '\n vendorString: %s\n' % gps.polystr(buf[12:43])
        s += ' productString: %s\n' % gps.polystr(buf[44:75])
        s += ' serialNumber: %s' % gps.polystr(buf[76:107])
        return s

    cfg_valdel_layers = {
        1: 'ram',
        2: 'bbr',
        4: 'flash',
        }

    cfg_valget_layers = {
        0: 'ram',
        1: 'bbr',
        2: 'flash',
        7: 'default',
        }

    cfg_valxxx_trans = {
        0: "Transactionless",
        1: "(Re)start Transaction",
        2: "Continue Transaction",
        3: "Apply and end Transaction",
        }

    def cfg_valdel(self, buf):
        """"UBX-CFG-VALDEL decode, Delete configuration items"""
        m_len = len(buf)

        if 4 > m_len:
            return "Bad Length %s" % m_len

        # this is a poll options, so does not set min protver

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u layer %#x transaction %#x reserved %u\n' % u
        s += ('  layers (%s) transaction (%s)' %
              (flag_s(u[1], self.cfg_valdel_layers),
               index_s(u[2], self.cfg_valxxx_trans)))

        m_len -= 4
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<L', buf, 4 + i * 4)
            item = self.cfg_by_key(u[0])
            s += ('\n    item: %s/%#x' % (item[0], u[0]))
            m_len -= 4
            i += 1
        return s

    def cfg_valget(self, buf):
        """"UBX-CFG-VALGET decode, Get configuration items"""
        m_len = len(buf)

        # version zero is a poll
        # version one is the response
        if 4 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u layer %u reserved %u,%u\n' % u
        s += '  layers (%s)' % index_s(u[1], self.cfg_valget_layers)

        m_len -= 4
        i = 0

        if 0 == u[0]:
            # this is a poll option, so does not set min protver
            while 0 < m_len:
                u = struct.unpack_from('<L', buf, 4 + i * 4)
                item = self.cfg_by_key(u[0])
                s += ('\n    item %s/%#x' % (item[0], u[0]))
                m_len -= 4
                i += 1
        else:
            # answer to poll
            # we are at least protver 27
            if 27 > opts['protver']:
                opts['protver'] = 27

            # duplicated in cfg_valset()
            m_len -= 4
            i = 4
            while 0 < m_len:
                u = struct.unpack_from('<L', buf, i)
                m_len -= 4
                i += 4
                item = self.cfg_by_key(u[0])
                cfg_type = self.item_to_type(item)

                size = cfg_type[0]
                frmat = cfg_type[1]
                flavor = cfg_type[2]
                v = struct.unpack_from(frmat, buf, i)
                s += ('\n    item %s/%#x val %s' % (item[0], u[0], v[0]))
                m_len -= size
                i += size

        return s

    def cfg_valset(self, buf):
        """"UBX-CFG-VALSET decode, Set configuration items"""
        m_len = len(buf)

        if 4 > m_len:
            return "Bad Length %s" % m_len

        # this is a poll option, so does not set min protver

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u layer %#x transaction %#x reserved %u\n' % u
        s += ('  layers (%s) transacion (%s)' %
              (flag_s(u[1], self.cfg_valdel_layers),
               index_s(u[2], self.cfg_valxxx_trans)))

        # duplicated in cfg_valset()
        m_len -= 4
        i = 4
        while 0 < m_len:
            u = struct.unpack_from('<L', buf, i)
            m_len -= 4
            i += 4
            item = self.cfg_by_key(u[0])
            cfg_type = self.item_to_type(item)

            size = cfg_type[0]
            frmat = cfg_type[1]
            flavor = cfg_type[2]
            v = struct.unpack_from(frmat, buf, i)
            s += ('\n    item %s/%#x val %s' % (item[0], u[0], v[0]))
            m_len -= size
            i += size
        return s

    cfg_ids = {0: {'str': 'PRT', 'dec': cfg_prt, 'name': 'UBX-CFG-PRT'},
               1: {'str': 'MSG', 'dec': cfg_msg, 'name': 'UBX-CFG-MSG'},
               2: {'str': 'INF', 'dec': cfg_inf, 'name': 'UBX-CFG-INF'},
               4: {'str': 'RST', 'dec': cfg_rst, 'name': 'UBX-CFG-RST'},
               6: {'str': 'DAT', 'name': 'UBX-CFG-DAT'},
               8: {'str': 'RATE', 'dec': cfg_rate, 'name': 'UBX-CFG-RATE'},
               9: {'str': 'CFG', 'dec': cfg_cfg, 'name': 'UBX-CFG-CFG'},
               0x11: {'str': 'RXM', 'name': 'UBX-CFG-RXM'},
               0x13: {'str': 'ANT', 'dec': cfg_ant, 'name': 'UBX-CFG-ANT'},
               0x16: {'str': 'SBAS', 'dec': cfg_sbas, 'name': 'UBX-CFG-SBAS'},
               0x17: {'str': 'NMEA', 'dec': cfg_nmea, 'name': 'UBX-CFG-NMEA'},
               0x1b: {'str': 'USB', 'dec': cfg_usb, 'name': 'UBX-CFG-USB'},
               0x1e: {'str': 'ODO', 'dec': cfg_odo, 'name': 'UBX-CFG-ODO'},
               0x23: {'str': 'NAVX5', 'dec': cfg_navx5,
                      'name': 'UBX-CFG-NAVX5'},
               0x24: {'str': 'NAV5', 'dec': cfg_nav5, 'name': 'UBX-CFG-NAV5'},
               0x31: {'str': 'TP5', 'dec': cfg_tp5, 'name': 'UBX-CFG-TP5'},
               0x34: {'str': 'RINV', 'dec': cfg_rinv, 'name': 'UBX-CFG-RINV'},
               0x39: {'str': 'ITFM', 'name': 'UBX-CFG-ITFM'},
               0x3b: {'str': 'PM2', 'name': 'UBX-CFG-PM2'},
               0x3d: {'str': 'TMODE2', 'dec': cfg_tmode2,
                      'name': 'UBX-CFG-TMODE2'},
               0x3e: {'str': 'GNSS', 'dec': cfg_gnss, 'name': 'UBX-CFG-GNSS'},
               0x47: {'str': 'LOGFILTER', 'name': 'UBX-CFG-LOGFILTER'},
               0x53: {'str': 'TXSLOT', 'name': 'UBX-CFG-TXSLOT'},
               0x57: {'str': 'PWR', 'name': 'UBX-CFG-PWR'},
               0x5c: {'str': 'HNR', 'name': 'UBX-CFG-HNR'},
               0x60: {'str': 'ESRC', 'name': 'UBX-CFG-ESRC'},
               0x61: {'str': 'DOSC', 'name': 'UBX-CFG-OSC'},
               0x62: {'str': 'SMGR', 'dec': cfg_smgr, 'name': 'UBX-CFG-SMGR'},
               0x69: {'str': 'GEOFENCE', 'name': 'UBX-CFG-GEOFENCE'},
               0x70: {'str': 'DGNSS', 'dec': cfg_dgnss,
                      'name': 'UBX-CFG-DGNSS'},
               0x71: {'str': 'TMODE3', 'name': 'UBX-CFG-TMODE3'},
               0x84: {'str': 'FIXSEED', 'name': 'UBX-CFG-FIXSEED'},
               0x85: {'str': 'DYNSEED', 'name': 'UBX-CFG-DYNSEED'},
               0x86: {'str': 'PMS', 'dec': cfg_pms, 'name': 'UBX-CFG-PMS'},
               0x8a: {'str': 'VALSET', 'dec': cfg_valset,
                      'name': 'UBX-CFG-VALSET'},
               0x8b: {'str': 'VALGET', 'dec': cfg_valget,
                      'name': 'UBX-CFG-VALGET'},
               0x8c: {'str': 'VALDEL', 'dec': cfg_valdel,
                      'name': 'UBX-CFG-VALDEL'},
               }

    def inf_debug(self, buf):
        """UBX-INF-DEBUG decode"""
        return ' Debug: ' + gps.polystr(buf)

    def inf_error(self, buf):
        """UBX-INF-ERROR decode"""
        return ' Error: ' + gps.polystr(buf)

    def inf_notice(self, buf):
        """UBX-INF-NOTICE decode"""
        return ' Notice: ' + gps.polystr(buf)

    def inf_test(self, buf):
        """UBX-INF-TET decode"""
        return ' Test: ' + gps.polystr(buf)

    def inf_warning(self, buf):
        """UBX-INF-WARNING decode"""
        return ' Warning: ' + gps.polystr(buf)

    inf_ids = {0x0: {'str': 'ERROR', 'dec': inf_error,
                     'name': 'UBX-INF-ERROR'},
               0x1: {'str': 'WARNING', 'dec': inf_warning,
                     'name': 'UBX-INF-WARNING'},
               0x2: {'str': 'NOTICE', 'dec': inf_notice,
                     'name': 'UBX-INF-NOTICE'},
               0x3: {'str': 'TEST', 'dec': inf_test,
                     'name': 'UBX-INF-TEST'},
               0x4: {'str': 'DEBUG', 'dec': inf_debug,
                     'name': 'UBX-INF-DEBUG'},
               }

    # UBX-LOG- ???
    # UBX-MGA- ???

    mon_comms_prot = {
        0: "UBX",
        1: "NMEA",
        2: "RTCM2",
        5: "RTCM3",
        255: "None",
        }

    def mon_comms(self, buf):
        """UBX-MON-COMMS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = ('version %u nPorts %u txErrors %#x reserved1 %u\n'
             'protIds %#x/%x/%x/%x' % u)
        s += (' (%s/%s/%s/%s)\n' %
              (index_s(u[4], self.mon_comms_prot),
               index_s(u[5], self.mon_comms_prot),
               index_s(u[6], self.mon_comms_prot),
               index_s(u[7], self.mon_comms_prot)))

        i = 0
        while m_len > (8 + (i * 40)):
            u = struct.unpack_from('<HHLBBHLBBHHHHHLLL', buf, (8 + (i * 40)))
            name = "%#x (%s)" % (u[0], index_s(u[0], self.port_ids1))
            if 0 < i:
                s += "\n"
            s += '  Port: %s\n' % name
            s += ('   txPending %u txBytes %u txUsage %u txPeakUsage %u\n'
                  '   rxPending %u rxBytes %u rxUsage %u rxPeakUsage %u\n'
                  '   overrunErrs %u msgs %u/%u/%u/%u reserved %x %x '
                  'skipped %u'
                  % u[1:])
            i += 1
        return s

    mon_gnss_supported_bits = {
        1: "GPS",
        2: "Glonass",
        4: "Beidou",
        8: "Galileo",
        }

    def mon_gnss(self, buf):
        """UBX-MON-GNSS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        s = ('   version %u supported %#x defaultGnss %#x enabled %#x\n'
             '   simultaneous %u reserved1 %u %u %u' % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ('\n     supported (%s)'
                  '\n     defaultGnss (%s)'
                  '\n     enabled (%s)' %
                  (flag_s(u[1], self.mon_gnss_supported_bits),
                   flag_s(u[2], self.mon_gnss_supported_bits),
                   flag_s(u[3], self.mon_gnss_supported_bits)))

        return s

    def mon_hw(self, buf):
        """UBX-MON-HW decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 60 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LLLLHHBBBBLBBBBBBBBBBBBBBBBBBBBLLL', buf, 0)
        s = ('   pinSel %#x pinBank %#x pinDir %#x pinVal %#x noisePerRMS %u\n'
             '   ageCnt %u aStatus %u aPower %u flags %#x reserved1 %u\n'
             '   usedMask %#x\n'
             '   VP %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n'
             '   jamInd %u reserved2 %u %u pinIrq %#x pullH %#x pullL %#x' % u)

        return s

    def mon_hw2(self, buf):
        """UBX-MON-HW2 decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<bBbBBBBBLLLLL', buf, 0)
        s = ('   ofsI %d magI %u ofsQ %d magQ %u cfgSource %u\n'
             '   reserved0 %u %u %u lowLevCfg %d reserved1 %u %u\n'
             '   postStatus %u reserved2 %u' % u)

        return s

    def mon_hw3(self, buf):
        """UBX-MON-HW3 decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 22 > m_len:
            return " Bad Length %s" % m_len

        # at least protver 27
        if 27 > opts['protver']:
            opts['protver'] = 27

        u = struct.unpack_from('<BBB', buf, 0)
        s = ('   version %u nPins %u flags %#x' % u)
        nPins = u[1]
        substr = buf[3:12]
        substr = substr.split(gps.polybytes('\0'))[0]
        s += ' hwVersion %s' % gps.polystr(substr)

        for i in range(22, 22 + (nPins * 6), 6):
            u = struct.unpack_from('<HHBB', buf, i)
            s += ('\n   pinId %4u pinMask %#5x VP %3u reserved2 %u' % u)

        return s

    def mon_io(self, buf):
        """UBX-MON-IO decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        i = 0
        s = ''
        while m_len > (i * 20):
            u = struct.unpack_from('<LLHHHHL', buf, i * 20)
            if 0 < i:
                s += "\n"
            if i in self.port_ids:
                name = self.port_ids[i]
            else:
                name = "Unk"
            s += ('  Port: %u (%s)\n' % (i, name))
            s += ('   rxBytes %u txBytes %u parityErrs %u framingErrs %u\n'
                  '   overrunErrs %u breakCond %u reserved %u' % u)
            i += 1
        return s

    def mon_msgpp(self, buf):
        """UBX-MON-MSGPP decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 120 > m_len:
            return " Bad Length %s" % m_len

        s = ''
        for i in range(1, 7):
            u = struct.unpack_from('<HHHHHHHH', buf, 0)
            s += " msg%u    %u %u %u %u %u %u %u %u\n" % ((i,) + u)

        u = struct.unpack_from('<LLLLLL', buf, 0)
        s += " skipped %u %u %u %u %u %u" % u
        return s

    mon_rf_jamming = {
        0: "Unk",
        1: "OK",
        2: "Warning",
        3: "Critical",
        }

    mon_rf_antstat = {
        0: "Init",
        1: "Unk",
        2: "OK",
        3: "Short",
        4: "Open",
        }

    mon_rf_antpwr = {
        0: "Off",
        1: "On",
        2: "Unk",
        }

    def mon_rf(self, buf):
        """UBX-MON-RF decode, RF Information"""

        # first seen in protver 27
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 4 > m_len:
            return " Bad Length %s" % m_len

        # at least protver 27
        if 27 > opts['protver']:
            opts['protver'] = 27

        u = struct.unpack_from('<BBBB', buf, 0)
        s = ' version %u nBlocks %u reserved1 %u %u' % u
        for i in range(0, u[1]):
            u = struct.unpack_from('<BBBBLBBBBHHBbBbBBBB', buf, 4 + (24 * i))
            s += ("\n   blockId %u flags x%x antStatus %u antPower %u "
                  "postStatus %u reserved2 %u %u %u %u"
                  "\n    noisePerMS %u agcCnt %u jamInd %u ofsI %d magI %u "
                  "ofsQ %d magQ %u"
                  "\n    reserved3 %u %u %u" % u)
            if VERB_DECODE <= opts['verbosity']:
                s += ('\n     jammingState (%s) antStatus (%s) antPower (%s)' %
                      (index_s(u[1] & 0x03, self.mon_rf_jamming),
                       index_s(u[2], self.mon_rf_antstat),
                       index_s(u[3], self.mon_rf_antpwr)))
        return s

    mon_patch_act = {
        1: "Active"
        }
    mon_patch_loc = {
        0: "eFuse",
        2: "ROM",
        4: "BBR",
        6: "File System",
        }

    def mon_patch(self, buf):
        """UBX-MON-PATCH decode, Output information about installed patches."""

        m_len = len(buf)

        if 0 == m_len:
            return " Poll request"

        if 4 > m_len:
            return " Bad Length %s" % m_len

        # first seen in protver 15
        u = struct.unpack_from('<HH', buf, 0)
        s = " version %u nEntries %u" % u

        for i in range(0, u[1]):
            u = struct.unpack_from('<LLLL', buf, 0)
            s = (" patchInfo x%x comparatorNumber %u patchAddress %u "
                 "patchData %u" % u)
            if VERB_DECODE <= opts['verbosity']:
                s += ('\n  patchInfo (%s, %s)' %
                      (flag_s(u[0] & 0x01, self.mon_patch_act),
                       index_s(u[0] & 0x06, self.mon_patch_loc)))

        return s

    def mon_rxbuf(self, buf):
        """UBX-MON-RXBUF decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 24 > m_len:
            return " Bad Length %s" % m_len

        rxbuf_name = {
           1: " pending   ",
           2: " usage     ",
           3: " peakUsage ",
           }

        s = ''
        for i in range(1, 4):
            u = struct.unpack_from('<HHHHHH', buf, 0)
            s += rxbuf_name[i] + "%u %u %u %u %u %u\n" % u
        return s

    def mon_rxr(self, buf):
        """UBX-MON-RXBUF decode, Receiver Status Information"""
        m_len = len(buf)

        if 1 > m_len:
            return " Bad Length %s" % m_len

        if 1 & buf[0]:
            s = " awake"
        else:
            s = " not awake"
        return s

    def mon_smgr(self, buf):
        """UBX-MON-SMGR decode, Synchronization manager status"""

        m_len = len(buf)

        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLHHBBBB', buf, 0)
        s = (" version %u reserved1 %u %u %u iTOW %u intOsc x%x extOsc x%x\n"
             " discOsc %u gnss x%x extInt0 x%x extInt1 %x%x" % u)
        return s

    def mon_txbuf(self, buf):
        """UBX-MON-TXBUF decode, Transmitter Buffer Status"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 > m_len:
            return " Bad Length %s" % m_len

        rxbuf_name = {
           1: " pending   ",
           2: " usage     ",
           3: " peakUsage ",
           }

        s = ''
        for i in range(1, 4):
            u = struct.unpack_from('<HHHHHH', buf, 0)
            s += rxbuf_name[i] + "%u %u %u %u %u %u\n" % u
        return s

    def mon_ver(self, buf):
        """UBX-MON-VER decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 40 > m_len:
            return " Bad Length %s" % m_len

        substr = buf.split(gps.polybytes('\0'))[0]
        s = ' swVersion %s\n' % gps.polystr(substr)
        substr = buf[30:39]
        substr = substr.split(gps.polybytes('\0'))[0]
        s += ' hwVersion %s' % gps.polystr(substr)
        # extensions??
        num_ext = int((m_len - 40) / 30)
        i = 0
        while i < num_ext:
            loc = 40 + (i * 30)
            substr = buf[loc:]
            substr = substr.split(gps.polybytes('\0'))[0]
            s += '\n extension: %s' % gps.polystr(substr)
            i += 1
        return s

    mon_ids = {2: {'str': 'IO', 'dec': mon_io, 'name': 'UBX-MON-IO'},
               4: {'str': 'VER', 'dec': mon_ver, 'name': 'UBX-MON-VER'},
               6: {'str': 'MSGPP', 'dec': mon_msgpp, 'name': 'UBX-MON-MSGPP'},
               7: {'str': 'RXBUF', 'dec': mon_rxbuf, 'name': 'UBX-MON-RXBUF'},
               8: {'str': 'TXBUF', 'dec': mon_txbuf, 'name': 'UBX-MON-TXBUF'},
               9: {'str': 'HW', 'dec': mon_hw, 'name': 'UBX-MON-HW'},
               0x0b: {'str': 'HW2', 'dec': mon_hw2, 'name': 'UBX-MON-HW2'},
               0x21: {'str': 'RXR', 'dec': mon_rxr, 'name': 'UBX-MON-RXR'},
               0x27: {'str': 'PATCH', 'dec': mon_patch,
                      'name': 'UBX-MON-PATCH'},
               0x28: {'str': 'GNSS', 'dec': mon_gnss, 'name': 'UBX-MON-GNSS'},
               0x2e: {'str': 'SMGR', 'dec': mon_smgr, 'name': 'UBX-MON-SMGR'},
               0x36: {'str': 'COMMS', 'dec': mon_comms,
                      'name': 'UBX-MON-COMMS'},
               0x37: {'str': 'HW3', 'dec': mon_hw3, 'name': 'UBX-MON-HW3'},
               0x38: {'str': 'RF', 'dec': mon_rf, 'name': 'UBX-MON-RF'},
               }

    def nav_clock(self, buf):
        """UBX-NAV-CLOCK decode, Clock Solution"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LllLL', buf, 0)
        return '  iTOW %u clkB %d clkD %d tAcc %u fAcc %u' % u

    nav_dgps_status = {
        0: "None",
        1: "PR+PRR correction",
        }

    def nav_dgps(self, buf):
        """UBX-NAV-DGPS decode, DGPS Data used for NAV"""

        # not present in protver 27+
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlhhBBH', buf, 0)
        s = (' iTOW %u age %d baseID %d basehealth %d numCh %u\n'
             ' status x%x reserved1 %u' % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n  status (%s)" %
                  index_s(u[5], self.nav_dgps_status))

        for i in range(0, u[4]):
            u = struct.unpack_from('<BbHff', buf, 16 + i * 12)
            s += ('\n  svid %3u flags x%2x ageC %u prc %f prcc %f' % u)
            if VERB_DECODE <= opts['verbosity']:
                s += ("\n   channel %u dgps %u" %
                      (u[1] & 0x0f, (u[1] >> 4) & 1))

        return s

    def nav_dop(self, buf):
        """UBX-NAV-DOP decode, Dilution of Precision"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 18 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<Lhhhhhhh', buf, 0)
        s = ('  iTOW %u gDOP %u pDOP %u tDOP %u vDOP %u\n'
             '  hDOP %u nDOP %u eDOP %u' % u)
        return s

    def nav_eoe(self, buf):
        """UBX-NAV-EOE decode, End Of Epoch"""
        m_len = len(buf)
        if 4 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<L', buf, 0)
        return ' iTOW %d' % u

    def nav_geofence(self, buf):
        """UBX-NAV-GEOFENCE decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBBB', buf, 0)
        s = '  iTOW:%d version %u status %u numFences %u combState %u' % u

        m_len -= 8
        i = 0
        while 1 < m_len:
            u = struct.unpack_from('<BB', buf, 8 + (i * 2))
            s += '\n  state %u reserved1 %u' % u
        return s

    def nav_hpposecef(self, buf):
        """UBX-NAV-POSECEF decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 < m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLlllbbbbL', buf, 0)
        return ('  version %u reserved1 %u %u %u iTOW %u\n'
                '  ecef: X %d Y %d Z %d\n'
                '  ecefHP: X %d Y %d Z %d\n'
                '  reserved2 %u pAcc %u' % u)

    def nav_hpposllh(self, buf):
        """UBX-NAV-HPPOSLLH decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 36 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLllllbbbbLL', buf, 0)
        return ('  version %u reserved1 %u %u %u iTOW %u\n'
                '  lon %d lat %d height %d hMSL %d\n'
                '  lonHp %d latHp %d heightHp %d hMSLHp %d\n'
                '  hAcc %u vAcc %u' % u)

    def nav_posecef(self, buf):
        """UBX-NAV-POSECEF decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlllL', buf, 0)
        return '  iTOW %u ecefX %d Y %d Z %d pAcc %u' % u

    def nav_posllh(self, buf):
        """UBX-NAV-POSLLH decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LllllLL', buf, 0)
        return ('  iTOW %u lon %d lat %d height %d\n'
                '  hMSL %d hAcc %u vAcc %u' % u)

    nav_pvt_valid = {
        1: "validDate",
        2: "ValidTime",
        4: "fullyResolved",
        8: "validMag",      # protver 27
        }

    # u-blox TIME ONLY is same as Surveyed
    nav_pvt_fixType = {
        0: 'None',
        1: 'Dead Reckoning',
        2: '2D',
        3: '3D',
        4: 'GPS+DR',
        5: 'Surveyed',
        }

    nav_pvt_flags = {
        1: "gnssFixOK",
        2: "diffSoln",
        0x20: "headVehValid",
        }

    nav_pvt_psm = {
        0: "n/a",
        1: "Enabled",
        2: "Acquisition",
        3: "Tracking",
        4: "Power Optimized Tracking",
        5: "Inactive",
        }

    # protver 27+
    nav_pvt_carr = {
        0: "None",
        1: "Floating",
        2: "Fixed",
        }

    def nav_pvt(self, buf):
        """UBX-NAV-PVT decode, Navigation Position Velocity Time Solution"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        # 84 bytes long in protver 14.
        # 92 bytes long in protver 15.
        if 84 > m_len:
            return " Bad Length %s" % m_len

        # flags2 is protver 27
        u = struct.unpack_from('<LHBBBBBBLlBBBBllllLLlllllLLHHHH', buf, 0)
        s = ('  iTOW %u time %u/%u/%u %2u:%2u:%2u valid x%x\n'
             '  tAcc %u nano %d fixType %u flags x%x flags2 x%x\n'
             '  numSV %u lon %d lat %d height %d\n'
             '  hMSL %d hAcc %u vAcc %u\n'
             '  velN %d velE %d velD %d gSpeed %d headMot %d\n'
             '  sAcc %u headAcc %u pDOP %u reserved1 %u %u %u' % u)

        if 92 <= m_len:
            # version 15
            u1 = struct.unpack_from('<lhH', buf, 81)
            s += ('\n  headVeh %d magDec %d magAcc %u' % u1)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n    valid (%s) fixType (%s)"
                  "\n    flags (%s)"
                  "\n    psmState (%s)"
                  "\n    carrSoln (%s)" %
                  (flag_s(u[7], self.nav_pvt_valid),
                   index_s(u[10], self.nav_pvt_fixType),
                   flag_s(u[11], self.nav_pvt_flags),
                   index_s((u[11] >> 2) & 0x0f, self.nav_pvt_psm),
                   index_s((u[11] >> 6) & 0x04, self.nav_pvt_carr)))
        return s

    nav_sat_qual = {
        0: "None",
        1: "Searching",
        2: "Acquired",
        3: "Detected",
        4: "Code and time locked",
        5: "Code, carrier and time locked",
        6: "Code, carrier and time locked",
        7: "Code, carrier and time locked",
        }

    nav_sat_health = {
        1: "Healthy",
        2: "Unhealthy",
        }

    nav_sat_orbit = {
        0: "None",
        1: "Ephemeris",
        2: "Almanac",
        3: "AssistNow Offline",
        4: "AssistNow Autonomous",
        5: "Other",
        6: "Other",
        7: "Other",
        }

    nav_sat_flags = {
        8: "svUsed",
        0x40: "diffCorr",
        0x80: "smoothed",
        0x800: "ephAvail",
        0x1000: "almAvail",
        0x2000: "anoAvail",
        0x4000: "aopAvail",
        0x10000: "sbasCorrUsed",
        0x20000: "rtcmCorrUsed",
        0x40000: "slasCorrUsed",
        0x100000: "prCorrUsed",
        0x200000: "crCorrUsed",
        0x400000: "doCorrUsed",
        }

    def nav_sat(self, buf):
        """UBX-NAV-SAT decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBBB', buf, 0)
        s = '  iTOW %u version %u numSvs %u reserved1 %u %u' % u

        for i in range(0, u[2]):
            u = struct.unpack_from('<BBBbhhL', buf, 8 + (i * 12))
            s += ('\n   gnssId %u svid %3u cno %2u elev %3d azim %3d prRes %6d'
                  ' flags x%x' % u)
            if VERB_DECODE <= opts['verbosity']:
                s += ("\n     flags (%s)"
                      "\n     qualityInd x%x (%s) health (%s)"
                      "\n     orbitSource (%s)" %
                      (flag_s(u[6], self.nav_sat_flags),
                       u[6] & 7, index_s(u[6] & 7, self.nav_sat_qual),
                       index_s((u[6] >> 4) & 3, self.nav_sat_health),
                       index_s((u[6] >> 8) & 7, self.nav_sat_orbit)))

        return s

    nav_sbas_mode = {
        0: "Disabled",
        1: "Enabled Integrity",
        2: "Enabled Testmode",
        }

    # sometimes U1 or I1
    nav_sbas_sys = {
        0: "WAAS",
        1: "EGNOS",
        2: "MSAS",
        3: "GAGAN",
        4: "SDCM",      # per ICAO Annex 10, v1, Table B-27
        16: "GPS",
        }

    nav_sbas_service = {
        1: "Ranging",
        2: "Corrections",
        4: "Integrity",
        8: "Testmode",
        }

    def nav_sbas(self, buf):
        """UBX-NAV-SBAS decode, SBAS Status Data"""

        # undocuemnted, but present in protver 27+
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 12 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBbBBBBB', buf, 0)
        s = (" iTOW %d geo %u mode x%x sys %d service x%x cnt %u "
             "reserved0 %u %u %u" % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n    mode (%s) sys (%s)"
                  "\n    service (%s)" %
                  (index_s(u[2], self.nav_sbas_mode),
                   index_s(u[3], self.nav_sbas_sys),
                   flag_s(u[4], self.nav_sbas_service)))

        for i in range(0, u[5]):
            u = struct.unpack_from('<BBBBBBhHh', buf, 12 + (i * 12))
            s += ("\n  svid %3d flags x%04x udre x%02x svSys %3d svService %2d"
                  " reserved2 %u"
                  "\n   prc %3d reserved3 %u ic %3d" % u)
            if VERB_DECODE <= opts['verbosity']:
                # where are flags and udre defined??
                s += ("\n   svSys (%s) svService (%s)" %
                      (index_s(u[3], self.nav_sbas_sys),
                       flag_s(u[4], self.nav_sbas_service)))

        return s

    def nav_sig(self, buf):
        """UBX-NAV-SIG decode, Signal Information"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBH', buf, 0)
        s = '  iTOW %u version %u numSigs %u reserved1 %u' % u

        numSigs = u[2]
        i = 0
        while i < numSigs:
            u = struct.unpack_from('<BBBBhBBBBHL', buf, 8 + (i * 16))
            s += ('\n   gnssId %u svId %u sigId %u freqId %u prRes %d cno %u '
                  'qualityInd %u\n'
                  '    corrSource %u ionoModel %u sigFlags %#x reserved2 %u' %
                  u)
            i += 1
        return s

    nav_sol_flags = {
        1: "GPSfixOK",
        2: "DiffSoln",
        4: "WKNSET",
        8: "TOWSET",
        }

    def nav_sol(self, buf):
        """UBX-NAV-SOL decode, Navigation Solution Information"""

        #  deprecated by u-blox
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 52 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlhBBlllLlllLHBBL', buf, 0)
        s = ('  iTOW %u fTOW %d week %d gpsFix %u flags x%x\n'
             '  ECEF X %d Y %d Z %d pAcc %u\n'
             '  VECEF X %d Y %d Z %d sAcc %u\n'
             '  pDOP %u reserved1 %u numSV %u reserved2 %u' % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   gpsfix (%s)"
                  "\n   flags (%s)" %
                  (index_s(u[3], self.nav_pvt_fixType),
                   flag_s(u[4], self.nav_sol_flags)))

        return s

    def nav_status(self, buf):
        """UBX-NAV-STATUS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBBBLL', buf, 0)
        return ('  iTOW:%d ms, fix:%d flags:%#x fixstat:%#x flags2:%#x\n'
                '  ttff:%d, msss:%d' % u)

    def nav_svin(self, buf):
        """UBX-NAV-SVIN decode, Survey-in data"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 40 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLLlllbbbBLLBB', buf, 0)
        return ('  version %u reserved1[%u %u %u] iTOW %u dur %u\n'
                '  meanX %d meanY %d meanZ %d\n'
                '  meanXHP %d meanYHP %d meanZHP %d reserved2 %u meanAcc %u\n'
                '  obs %u valid %u active %u' % u)

    def nav_svinfo(self, buf):
        """UBX-NAV-SVINFO decode"""
        m_len = len(buf)
        if 0 == m_len:
            return "Poll request"

        if 8 > m_len:
            return "Bad Length %s" % m_len

        u = struct.unpack_from('<Lbb', buf, 0)
        s = ' iTOW:%d ms, numCh:%d globalFlags:%d' % u

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBBBBbhl', buf, 8 + i * 12)
            s += ('\n  chn %3d svid %3d flags %#0.2x quality %#x cno %2d'
                  ' elev %3d azim %3d prRes %6d' % u)
            if 0 < u[2]:
                s += '\n   '
                if 1 & u[2]:
                    s += 'svUsed '
                if 2 & u[2]:
                    s += 'diffCorr '
                if 4 & u[2]:
                    s += 'orbitAvail '
                if 8 & u[2]:
                    s += 'orbitEph '
                if 0x10 & u[2]:
                    s += 'unhealthy '
                if 0x20 & u[2]:
                    s += 'orbitAlm '
                if 0x40 & u[2]:
                    s += 'orbitAop '
                if 0x80 & u[2]:
                    s += 'smoothed '
            m_len -= 12
            i += 1

        return s

    nav_time_valid = {
        1: "towValid",
        2: "weekValid",
        4: "leapValid",
        }

    def nav_timebds(self, buf):
        """UBX-NAV-TIMEBDS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return ".Bad Length %s" % m_len

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d SOW %d fSOW %d week %d leapS %d\n"
             "  Valid %#x tAcc %d" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timegal(self, buf):
        """UBX-NAV-TIMEGAL decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d galTOW %d fGalTow %d galWno %d leapS %d\n"
             "  Valid x%x, tAcc %d" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timeglo(self, buf):
        """UBX-NAV-TIMEGLO decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LLlhbBL', buf, 0)
        s = ("  iTOW %d TOD %d fTOD %d Nt %d  N4 %d\n"
             "  Valid x%x tAcc %d" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   valid (%s)" %
                  (flag_s(u[5], self.nav_time_valid)))
        return s

    def nav_timegps(self, buf):
        """UBX-NAV-TIMEGPS decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlhbBL', buf, 0)
        s = "  iTOW %u fTOW %u week %d leapS %d valid x%x tAcc %d" % u

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   valid (%s)" %
                  (flag_s(u[4], self.nav_time_valid)))
        return s

    nav_timels_src = {
        0: "Default",
        1: "GPS/GLONASS derived",
        2: "GPS",
        3: "SBAS",
        4: "BeiDou",
        5: "Galileo",
        6: "Aided data",
        7: "Configured",
        }

    nav_timels_src1 = {
        0: "None",
        2: "GPS",
        3: "SBAS",
        4: "BeiDou",
        5: "Galileo",
        6: "GLONASS",
        }

    nav_timels_valid = {
        1: "validCurrLs",
        2: "validTimeToLsEvent",
        }

    def nav_timels(self, buf):
        """UBX-NAV-TIMELS decode, Leap second event information"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 24 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LBBBBBbBblHHBBBB', buf, 0)
        s = ('  iTOW %u version %u reserved2 %u %u %u srcOfCurrLs %u\n'
             '  currLs %d srcOfLsChange %u lsChange %d timeToLsEvent %d\n'
             '  dateOfLsGpsWn %u dateOfLsGpsDn %u reserved2 %u %u %u\n'
             '  valid x%x' % u)
        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   srcOfCurrLs (%s) srcOfLsChange (%s)"
                  "\n   valid (%s)" %
                  (index_s(u[5], self.nav_timels_src),
                   index_s(u[7], self.nav_timels_src1),
                   flag_s(u[15], self.nav_timels_valid)))
        return s

    nav_timeutc_valid = {
        1: "validTOW",
        2: "validWKN",
        4: "validUTC",
        }

    def nav_timeutc(self, buf):
        """UBX-NAV-TIMEUTC decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LLlHbbbbbB', buf, 0)
        s = ("  iTOW %d tAcc %d nano %d Time  %d/%d/%d %d:%d:%d\n"
             "  valid x%x" % u)

        if VERB_DECODE <= opts['verbosity']:
            s += ("\n   valid (%s) utcStandard (%s)" %
                  (flag_s(u[9], self.nav_timeutc_valid),
                   index_s(u[9] >> 4, self.utc_std)))
        return s

    def nav_velecef(self, buf):
        """UBX-NAV-VELECEF decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 20 != m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlllL', buf, 0)
        return '  iTOW %d ecef: VX %d VY %d VZ %d vAcc:%u' % u

    def nav_velned(self, buf):
        """UBX-NAV-VELNED decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 36 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlllLLlLL', buf, 0)
        return ('  iTOW %u vel: N %d E %d D %d speed %u\n'
                '  gspeed %u heading %d sAcc %u cAcc %u' % u)

    nav_ids = {1: {'str': 'POSECEF', 'dec': nav_posecef,
                   'name': 'UBX-NAV-POSECEF'},
               2: {'str': 'POSLLH', 'dec': nav_posllh,
                   'name': 'UBX-NAV-POSLLH'},
               3: {'str': 'STATUS', 'dec': nav_status,
                   'name': 'UBX-NAV-STATUS'},
               0x4: {'str': 'DOP', 'dec': nav_dop, 'name': 'UBX-NAV-DOP'},
               0x5: {'str': 'ATT', 'name': 'UBX-NAV-ATT'},
               0x6: {'str': 'SOL', 'dec': nav_sol, 'name': 'UBX-NAV-SOL'},
               0x7: {'str': 'PVT', 'dec': nav_pvt, 'name': 'UBX-NAV-PVT'},
               0x9: {'str': 'ODO', 'name': 'UBX-NAV-ODO'},
               0x10: {'str': 'RESETODO', 'name': 'UBX-NAV-RESETODO'},
               0x11: {'str': 'VELECEF', 'dec': nav_velecef,
                      'name': 'UBX-NAV-VELECEF'},
               0x12: {'str': 'VELNED', 'dec': nav_velned,
                      'name': 'UBX-NAV-VELNED'},
               0x13: {'str': 'HPPOSECEF', 'dec': nav_hpposecef,
                      'name': 'UBX-NAV-HPPOSECEF'},
               0x14: {'str': 'HPPOSLLH', 'dec': nav_hpposllh,
                      'name': 'UBX-NAV-HPPOSLLH'},
               0x20: {'str': 'TIMEGPS', 'dec': nav_timegps,
                      'name': 'UBX-NAV-TIMEGPS'},
               0x21: {'str': 'TIMEUTC', 'dec': nav_timeutc,
                      'name': 'UBX-NAV-TIMEUTC'},
               0x22: {'str': 'CLOCK', 'dec': nav_clock,
                      'name': 'UBX-NAV-CLOCK'},
               0x23: {'str': 'TIMEGLO', 'dec': nav_timeglo,
                      'name': 'UBX-NAV-TIMEGLO'},
               0x24: {'str': 'TIMEBDS', 'dec': nav_timebds,
                      'name': 'UBX-NAV-TIMEBDS'},
               0x25: {'str': 'TIMEGAL', 'dec': nav_timegal,
                      'name': 'UBX-NAV-TIMEGAL'},
               0x26: {'str': 'TIMELS', 'dec': nav_timels,
                      'name': 'UBX-NAV-TIMELS'},
               0x30: {'str': 'SVINFO', 'dec': nav_svinfo,
                      'name': 'UBX-NAV-SVINFO'},
               0x31: {'str': 'DGPS', 'dec': nav_dgps, 'name': 'UBX-NAV-DGPS'},
               0x32: {'str': 'SBAS', 'dec': nav_sbas, 'name': 'UBX-NAV-SBAS'},
               0x34: {'str': 'ORB', 'name': 'UBX-NAV-ORB'},
               0x35: {'str': 'SAT', 'dec': nav_sat, 'name': 'UBX-NAV-SAT'},
               0x39: {'str': 'GEOFENCE', 'dec': nav_geofence,
                      'name': 'UBX-NAV-GEOFENCE'},
               0x3B: {'str': 'SVIN', 'dec': nav_svin, 'name': 'UBX-NAV-SVIN'},
               0x3C: {'str': 'RELPOSNED', 'name': 'UBX-NAV-RELPOSNED'},
               0x43: {'str': 'SIG', 'dec': nav_sig, 'name': 'UBX-NAV-SIG'},
               0x60: {'str': 'AOPSTATUS', 'name': 'UBX-NAV-AOPSTATUS'},
               0x61: {'str': 'EOE', 'dec': nav_eoe, 'name': 'UBX-NAV-EOE'},
               }

    # used for RTCM3 rate config
    rtcm_ids = {5: {'str': '1005'},
                0x4a: {'str': '1074'},
                0x4d: {'str': '1077'},
                0x54: {'str': '1084'},
                0x57: {'str': '1087'},
                0x61: {'str': '1097'},
                0x7c: {'str': '1124'},
                0x7f: {'str': '1127'},
                0xe6: {'str': '1230'},
                0xfd: {'str': '4072-1'},
                0xfe: {'str': '4072-0'},
                }

    # used for NMEA rate config
    nmea_ids = {0: {'str': 'GGA'},
                1: {'str': 'GLL'},
                2: {'str': 'GSA'},
                3: {'str': 'GSV'},
                4: {'str': 'RMC'},
                5: {'str': 'VTG'},
                6: {'str': 'GRS'},
                7: {'str': 'GST'},
                8: {'str': 'ZDA'},
                9: {'str': 'GBS'},
                0x0a: {'str': 'DTM'},
                0x0d: {'str': 'GNS'},
                0x0f: {'str': 'VLW'},
                0x40: {'str': 'GPQ'},
                0x41: {'str': 'TXT'},
                0x42: {'str': 'GNQ'},
                0x43: {'str': 'GLQ'},
                0x44: {'str': 'GBQ'},
                0x45: {'str': 'GAQ'},
                }

    def rxm_measx(self, buf):
        """UBX-RXM-RAW decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 44 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBLLLLLHHHHHBBLL', buf, 0)
        s = (' version %u reserved1 %u %u %u gpsTOW %u gloTOW %u\n'
             ' bdsTOW %u reserved2 %u qzssTOW %u gpsTOWacc %u\n'
             ' gloTOWacc %u bdsTOWacc %u reserved3 %u qzssTOWacc %u\n'
             ' numSV %u flags %#x reserved4 %u %u' % u)

        m_len -= 44
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBBBllHHLBBH', buf, 44 + i * 24)
            s += ('\n  gnssId %u svId %u cNo %u mpathIndic %u DopplerMS %d\n'
                  '    dopplerHz %d wholeChips %u fracChips %u codephase %u\n'
                  '    intCodePhase %u pseudoRangeRMSErr %u reserved5 %u' % u)
            m_len -= 24
            i += 1

        return s

    def rxm_raw(self, buf):
        """UBX-RXM-RAW decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<lhBB', buf, 0)
        s = ' iTOW %d weeks %d numSV %u res1 %u' % u

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<ddfBbbB', buf, 8 + i * 24)
            s += ('\n  cpMes %f prMes %f doMes %f sv %d mesQI %d\n'
                  '     eno %d lli %d' % u)
            m_len -= 24
            i += 1

        return s

    rxm_rawx_recs = {
        1: "leapSec",
        2: "clkReset",
        }

    def rxm_rawx(self, buf):
        """UBX-RXM-RAWX decode"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        # version not here before protver 18, I hope it is zero.
        u = struct.unpack_from('<dHbBBBBB', buf, 0)
        s = (' rcvTow %.3f week %u leapS %d numMeas %u recStat %#x'
             ' version %u\n'
             ' reserved1[2] %#x %#x\n  recStat (' % u)
        s += flag_s(u[4], self.rxm_rawx_recs) + ')'

        m_len -= 16
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<ddfBBBBHBBBBB', buf, 16 + i * 32)
            s += ('\n  prmes %.3f cpMes %.3f doMes %f\n'
                  '   gnssId %u svId %u sigId %u freqId %u locktime %u '
                  'cno %u\n'
                  '   prStdev %u cpStdev %u doStdev %u trkStat %u' % u)

            if VERB_DECODE < opts['verbosity']:
                s += '\n      (%s)' % self.gnss_s(u[3], u[4], u[5])

            m_len -= 32
            i += 1
        return s

    def rxm_rlm(self, buf):
        """UBX-RXM-RLM decode, Galileo SAR RLM report"""
        m_len = len(buf)

        if 16 > m_len:
            return " Bad Length %s" % m_len

        # common to Short-RLM and Long-RLM report
        u = struct.unpack_from('<BBBBLLB', buf, 0)
        s = (" version %u type %u svId %u reserved1 %u beacon x%x %x "
             " message %u" % u)
        if 16 == m_len:
            # Short-RLM report
            u = struct.unpack_from('<BBB', buf, 13)
            s += "\n params %u %u reserved2 %u" % u
        elif 28 == m_len:
            # Long-RLM report
            u = struct.unpack_from('<BBBBBBBBBBBBBBB', buf, 13)
            s += ("\n params %u %u %u %u %u %u %u %u %u %u %u %u"
                  "\n reserved2 %u %u %u" % u)

        return s

    def rxm_rtcm(self, buf):
        """UBX-RXM-RTCM decode, RTCM Input Status"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBHHH', buf, 0)
        s = " version %u flags x%x subtype %u refstation %u msgtype %u" % u
        return s

    def rxm_sfrb(self, buf):
        """UBX-RXM-SFRB decode, Subframe Buffer"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 42 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBLLLLLLLLLL', buf, 0)
        s = (' chn %d s svid %3d\n'
             ' dwrd %08x %08x %08x %08x %08x\n'
             '      %08x %08x %08x %08x %08x' % u)

        return s

    # decode GPS subframe 5, pages 1 to 24,
    # and subframe 4, pages 2 to 5, and 7 to 10
    def almanac(self, words):
        """Decode GPS Almanac"""

        # Note: not really tested!  What to test against?
        # [1] Section 20.3.3.5, Figure 20-1 Sheet 4, and Table 20-VI.

        # e = Eccentricity
        # toa = Almanac reference time
        # deltai =
        # Omegadot = Rate of Right Ascension
        # SVH = SV Health
        # sqrtA = Square Root of the Semi-Major Axis
        # Omega0 = Longitude of Ascending Node of Orbit Plane at Weekly Epoch
        # omega = Argument of Perigee
        # M0 = Mean Anomaly at Reference Time
        # af0 = SV Clock Bias Correction Coefficient
        # af1 = SV Clock Drift Correction Coefficient
        s = "    Almanac"
        s += ("\n    e %e toa %u deltai %e Omegadot %e"
              "\n    SVH x%x sqrtA %.6f Omega0 %e omega %e"
              "\n    M0 %e af0 %e af1 %e" %
              (unpack_u16(words[2], 6) * (2 ** -21),
               unpack_u8(words[3], 22) * (2 ** 12),
               unpack_s16(words[3], 6) * (2 ** -19),
               unpack_s16(words[4], 14) * (2 ** -38),
               unpack_u8(words[4], 6),
               unpack_u24(words[5], 6) * (2 ** -11),     # sqrtA
               unpack_s24(words[6], 6) * (2 ** -23),
               unpack_s24(words[7], 6) * (2 ** -23),
               unpack_s24(words[8], 6) * (2 ** -23),     # M0
               unpack_s11s(words[9]) * (2 ** -20),
               unpack_s11(words[9], 11) * (2 ** -38)))
        return s

    cnav_msgids = {
        10: "Ephemeris 1",
        11: "Ephemeris 2",
        12: "Reduced Almanac",
        13: "Clock Differential Correction",
        14: "Ephemeris Differential Correction",
        15: "Text",
        30: "Clock, IONO & Group Delay",
        31: "Clock & Reduced Almanac",
        32: "Clock & EOP",
        33: "Clock & UTC",
        34: "Clock & Differential Correction",
        35: "Clock & GGTO",
        36: "Clock & Text",
        37: "Clock & Midi Almanac",
        }

    # map subframe 4 SV ID to Page number
    # IS-GPS-200J Table 20-V
    sbfr4_svid_page = {
        57: 1,
        25: 2,
        26: 3,
        27: 4,
        28: 5,
        57: 6,
        29: 7,
        30: 8,
        31: 9,
        32: 10,
        57: 11,
        62: 12,
        52: 13,
        53: 14,
        54: 15,
        57: 16,
        55: 17,
        56: 18,
        58: 19,
        59: 20,
        57: 21,
        60: 22,
        61: 23,
        62: 24,
        63: 25,
        }

    # map subframe 5 SV ID to Page number
    # IS-GPS-200J Table 20-V
    sbfr5_svid_page = {
        1: 1,
        2: 2,
        3: 3,
        4: 4,
        5: 5,
        6: 6,
        7: 7,
        8: 8,
        9: 9,
        10: 10,
        11: 11,
        12: 12,
        13: 13,
        14: 14,
        15: 15,
        16: 16,
        17: 17,
        18: 18,
        19: 19,
        20: 20,
        21: 21,
        22: 22,
        23: 23,
        24: 24,
        51: 25,
        }

    # URA Index to URA meters
    ura_meters = {
        0: "2.40 m",
        1: "3.40 m",
        2: "4.85 m",
        3: "6.85 m",
        4: "9.65 m",
        5: "13.65 m",
        6: "24.00 m",
        7: "48.00 m",
        8: "96.00 m",
        9: "192.00 m",
        10: "384.00 m",
        11: "768.00 m",
        12: "1536.00 m",
        13: "3072.00 m",
        14: "6144.00 m",
        15: "Unk",
        }

    codes_on_l2 = {
        0: "Invalid",
        1: "P-code ON",
        2: "C/A-code ON",
        3: "Invalid",
        }

    def rxm_sfrbx(self, buf):
        """UBX-RXM-SFRBX decode, Broadcast Navigation Data Subframe"""

        # The way u-blox packs the subfram data is perverse, and
        # undocuemnted.  Even more perverse than native subframes.
        m_len = len(buf)

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBB', buf, 0)
        svId = u[1]
        s = (' gnssId %u svId %3u reserved1 %u freqId %u numWords %u\n'
             '  reserved2 %u version %u reserved3 %u\n' % u)
        s += '    dwrd'
        words = ()
        for i in range(8, 8 + (u[4] * 4), 4):
            u1 = struct.unpack_from('<L', buf, i)
            s += " %08x" % u1
            words += (u1[0],)

        if VERB_DECODE <= opts['verbosity']:

            if 0 == u[0]:
                # GPS
                preamble = words[0] >> 24
                if 0x8b == preamble:
                    # CNAV
                    msgid = (words[0] >> 12) & 0x3f
                    s += ("\n  CNAV: preamble %#x PRN %u msgid %d (%s)\n" %
                          (preamble, (words[0] >> 18) & 0x3f,
                           msgid, index_s(msgid, self.cnav_msgids)))

                else:
                    # LNAV-L
                    preamble = words[0] >> 22
                    subframe = (words[1] >> 8) & 0x07
                    s += ("\n  LNAV-L: preamble %#x TLM %#x ISF %u" %
                          (preamble, (words[0] >> 8) & 0xffff,
                           1 if (words[0] & 0x40) else 0))

                    s += ("\n  TOW %u AF %u ASF %u Subframe %u" %
                          (unpack_u8(words[1], 13) * 6,
                           1 if (words[0] & 0x1000) else 0,
                           1 if (words[0] & 0x800) else 0,
                           subframe))

                    if 1 == subframe:
                        # not well validated decode, possibly wrong...
                        # [1] Figure 20-1 Sheet 1, Table 20-I
                        # WN = GPS week number
                        # TGD = Group Delay Differential
                        # tOC = Time of Clock
                        # af0 = SV Clock Bias Correction Coefficient
                        # af1 = SV Clock Drift Correction Coefficient
                        # af2 = Drift Rate Correction Coefficient
                        ura = (words[2] >> 14) & 0x0f
                        c_on_l2 = (words[2] >> 18) & 0x03
                        iodc = ((((words[2] >> 6) & 0x03) << 8) |
                                (words[7] >> 24) & 0xff)
                        s += ("\n   WN %u Codes on L2 %u (%s) URA %u (%s) "
                              "SVH %#04x IODC %u" %
                              (words[2] >> 20,
                               c_on_l2, index_s(c_on_l2, self.codes_on_l2),
                               ura, index_s(ura, self.ura_meters),
                               (words[2] >> 8) & 0x3f, iodc))
                        # tOC = Clock Data Reference Time of Week
                        s += ("\n   L2 P DF %u TGD %e tOC %u\n"
                              "   af2 %e af1 %e af0 %e" %
                              ((words[2] >> 29) & 0x03,
                               unpack_s8(words[6], 6) * (2 ** -31),
                               unpack_u16(words[7], 6) * 16,
                               unpack_s8(words[8], 22) * (2 ** -55),
                               unpack_s16(words[8], 6) * (2 ** -43),
                               unpack_s22(words[9], 8) * (2 ** -31)))

                    elif 2 == subframe:
                        # not well validated decode, possibly wrong...
                        # [1] Figure 20-1 Sheet 1, Tables 20-II and 20-III
                        # IODE = Issue of Data (Ephemeris)
                        # Crs = Amplitude of the Sine Harmonic Correction
                        #       Term to the Orbit Radius
                        # Deltan = Mean Motion Difference From Computed Value
                        # M0 = Mean Anomaly at Reference Time
                        # Cuc = Amplitude of the Cosine Harmonic Correction
                        #       Term to the Argument of Latitude
                        # e = Eccentricity
                        # Cus = Amplitude of the Sine Harmonic Correction Term
                        #       to the Argument of Latitude
                        # sqrtA = Square Root of the Semi-Major Axis
                        # tOE = Reference Time Ephemeris
                        s += ("\n   IODE %u Crs %e Deltan %e M0 %e"
                              "\n   Cuc %e e %e Cus %e sqrtA %f tOE %u" %
                              (unpack_u8(words[2], 22),
                               unpack_s16(words[2], 6) * (2 ** -5),
                               unpack_s16(words[3], 14) * (2 ** -43),
                               # M0
                               unpack_s32s(words[4], words[3]) * (2 ** -31),
                               unpack_s16(words[5], 14) * (2 ** -29),
                               unpack_u32s(words[6], words[5]) * (2 ** -33),
                               unpack_s16(words[7], 14) * (2 ** -29),
                               unpack_u32s(words[8], words[7]) * (2 ** -19),
                               unpack_u16(words[9], 14) * 16))

                    elif 3 == subframe:
                        # not well validated decode, possibly wrong...
                        # [1] Figure 20-1 Sheet 3, Table 20-II, Table 20-III
                        # Cic = Amplitude of the Cosine Harmonic Correction
                        #       Term to the Angle of Inclination
                        # Omega0 = Longitude of Ascending Node of Orbit
                        #          Plane at Weekly Epoch
                        # Cis = Amplitude of the Sine Harmonic Correction
                        #       Term to the Orbit Radius
                        # i0 = Inclination Angle at Reference Time
                        # Crc = Amplitude of the Cosine Harmonic Correction
                        #       Term to the Orbit Radius
                        # omega = Argument of Perigee
                        # Omegadot = Rate of Right Ascension
                        # IODE = Issue of Data (Ephemeris)
                        # IODT = Rate of Inclination Angle
                        s += ("\n   Cic %e Omega0 %e Cis %e i0 %e"
                              "\n   Crc %e omega %e Omegadot %e"
                              "\n   IDOE %u IDOT %e" %
                              (unpack_s16(words[2], 14) * (2 ** -29),
                               unpack_s32s(words[3], words[2]) * (2 ** -31),
                               unpack_s16(words[4], 14) * (2 ** -29),
                               unpack_s32s(words[5], words[4]) * (2 ** -31),
                               # Crc
                               unpack_s16(words[6], 14) * (2 ** -5),
                               unpack_s32s(words[7], words[6]) * (2 ** -31),
                               # Omegadot
                               unpack_s24(words[8], 6) * (2 ** -43),
                               unpack_u8(words[9], 22),
                               unpack_s14(words[9], 8) * (2 ** -43)))

                    elif 4 == subframe:
                        # all data in subframe 4 is "reserved",
                        # except for pages 13, 18, 15
                        # as of 2018, dataid is always 1.
                        svid = (words[2] >> 22) & 0x3f
                        if 0 < svid:
                            page = index_s(svid, self.sbfr4_svid_page)
                        else:
                            # page of zero means the svId that sent it.
                            page = "%d/Self" % svId

                        s += ("\n   dataid %u svid %u (page %s)\n" %
                              (words[2] >> 28, svid, page))

                        if (((2 <= page and 5 >= page) or
                             (5 <= page and 10 >= page))):
                            s += self.almanac(words)
                        elif 13 == page:
                            s += "    NWCT"
                        elif 17 == page:
                            s += ("    Special messages: " +
                                  chr((words[2] >> 14) & 0xff) +
                                  chr((words[2] >> 6) & 0xff) +
                                  chr((words[3] >> 22) & 0xff) +
                                  chr((words[3] >> 14) & 0xff) +
                                  chr((words[3] >> 6) & 0xff) +
                                  chr((words[4] >> 22) & 0xff) +
                                  chr((words[4] >> 14) & 0xff) +
                                  chr((words[4] >> 6) & 0xff) +
                                  chr((words[5] >> 22) & 0xff) +
                                  chr((words[5] >> 14) & 0xff) +
                                  chr((words[5] >> 6) & 0xff) +
                                  chr((words[6] >> 22) & 0xff) +
                                  chr((words[6] >> 14) & 0xff) +
                                  chr((words[6] >> 6) & 0xff) +
                                  chr((words[7] >> 22) & 0xff) +
                                  chr((words[7] >> 14) & 0xff) +
                                  chr((words[7] >> 6) & 0xff) +
                                  chr((words[8] >> 22) & 0xff) +
                                  chr((words[8] >> 14) & 0xff) +
                                  chr((words[8] >> 6) & 0xff) +
                                  chr((words[9] >> 22) & 0xff) +
                                  chr((words[9] >> 14) & 0xff))

                        elif 18 == page:
                            s += "    Ionospheric and UTC data"
                        elif 25 == page:
                            s += "    A/S flags"
                        else:
                            s += "    Reserved"

                    elif 5 == subframe:
                        svid = (words[2] >> 22) & 0x3f
                        page = index_s(svid, self.sbfr5_svid_page)

                        s += ("\n   dataid %u svid %u (page %s)\n" %
                              (words[2] >> 28, svid, page))

                        if 1 <= page and 24 >= page:
                            s += self.almanac(words)
                        elif 25 == page:
                            s += "    A/S flags"
                        else:
                            s += "    Reserved"

        return s

    def rxm_svsi(self, buf):
        """UBX-RXM-SVSI decode, SV Status Info"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 8 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LhBB', buf, 0)
        s = ' iTOW %d week %d numVis %d numSV %d' % u

        m_len -= 8
        i = 0
        while 0 < m_len:
            u = struct.unpack_from('<BBhbB', buf, 8 + i * 6)
            s += '\n  svid %3d svFlag %#x azim %3d elev % 3d age %3d' % u
            m_len -= 6
            i += 1

        return s

    rxm_ids = {0x10: {'str': 'RAW', 'dec': rxm_raw,
                      'name': 'UBX-RXM-RAW'},      # obsolete
               0x11: {'str': 'SFRB', 'dec': rxm_sfrb,
                      'name': 'UBX-RXM-SFRB'},
               0x13: {'str': 'SFRBX', 'dec': rxm_sfrbx,
                      'name': 'UBX-RXM-SFRBX'},
               0x14: {'str': 'MEASX', 'dec': rxm_measx,
                      'name': 'UBX-RXM-MEASX'},
               0x15: {'str': 'RAWX', 'dec': rxm_rawx, 'name': 'UBX-RXM-RAWX'},
               0x20: {'str': 'SVSI', 'dec': rxm_svsi, 'name': 'UBX-RXM-SVSI'},
               0x32: {'str': 'RTCM', 'dec': rxm_rtcm, 'name': 'UBX-RXM-RTCM'},
               0x41: {'str': 'PMREQ', 'name': 'UBX-RXM-PMREQ'},
               0x59: {'str': 'RLM', 'dec': rxm_rlm, 'name': 'UBX-RXM-RLM'},
               0x61: {'str': 'IMES', 'name': 'UBX-RXM-IMES'},
               }

    # UBX-SEC-
    def sec_uniqid(self, buf):
        """UBX-SEC_UNIQID decode Unique chip ID"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 9 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBBBBBBBB', buf, 0)
        s = (' version %u reserved %u %u %u uniqueId %#02x%02x%02x%02x%02x'
             % u)
        return s

    sec_ids = {0x01: {'str': 'SIGN', 'name': 'UBX-SEC-SIGN'},
               0x03: {'str': 'UNIQID', 'dec': sec_uniqid,
                      'name': 'UBX-SEC-UNIQID'},
               }

    # UBX-TIM-
    def tim_svin(self, buf):
        """UBX-TIM-SVIN decode, Survey-in data"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LlllLLBB', buf, 0)
        s = ('  dur %u meanX %d meanY %d meanZ %d meanV %u\n'
             '  obs %u valid %u active %u' % u)
        return s

    def tim_tm2(self, buf):
        """UBX-TIM-TM2 decode, Time mark data"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 28 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<BBHHHLLLLL', buf, 0)
        s = ('  ch %u flags %#x count %u wnR %u wnF %u\n'
             '  towMsR %u towSubMsR %u towMsF %u towSubMsF %u accEst %u\n' % u)
        return s

    def tim_tp(self, buf):
        """UBX-TIM-TP decode, Time Pulse Timedata"""
        m_len = len(buf)
        if 0 == m_len:
            return " Poll request"

        if 16 > m_len:
            return " Bad Length %s" % m_len

        u = struct.unpack_from('<LLlHbb', buf, 0)
        s = ('  towMS %u towSubMS %u qErr %d week %d\n'
             '  flags %#x refInfo %#x\n   flags  ' % u)

        if 0x01 & u[4]:
            s += "timeBase is UTC, "
        else:
            s += "timeBase is GNSS, "
        if 0x02 & u[4]:
            s += "UTC available, "
        else:
            s += "UTC not available, "

        raim = (u[4] >> 2) & 0x03
        if 0 == raim:
            s += "RAIM not available"
        elif 1 == raim:
            s += "RAIM not active"
        elif 2 == raim:
            s += "RAIM active"
        else:
            s += "RAIM ??"
        return s

    tim_ids = {1: {'str': 'TP', 'dec': tim_tp, 'name': 'UBX-TIM-TP'},
               3: {'str': 'TM2', 'dec': tim_tm2, 'name': 'UBX-TIM-TM2'},
               4: {'str': 'SVIN', 'dec': tim_svin, 'name': 'UBX-TIM-SVIN'},
               6: {'str': 'VRFY', 'name': 'UBX-TIM-VRFY'},
               0x11: {'str': 'DOSC', 'name': 'UBX-TIM-DOSC'},
               0x12: {'str': 'TOS', 'name': 'UBX-TIM-TOS'},
               0x13: {'str': 'SMEAS', 'name': 'UBX-TIM-SMEAS'},
               0x15: {'str': 'VCOCAL', 'name': 'UBX-TIM-VCOCAL'},
               0x16: {'str': 'FCHG', 'name': 'UBX-TIM-FCHG'},
               0x17: {'str': 'HOC', 'name': 'UBX-TIM-HOC'},
               }

    # UBX-UPD- ??

    classes = {
        0x01: {'str': 'NAV', 'ids': nav_ids},
        0x02: {'str': 'RXM', 'ids': rxm_ids},
        0x04: {'str': 'INF', 'ids': inf_ids},
        0x05: {'str': 'ACK', 'ids': ack_ids},
        0x06: {'str': 'CFG', 'ids': cfg_ids},
        0x09: {'str': 'UPD'},
        0x0A: {'str': 'MON', 'ids': mon_ids},
        0x0B: {'str': 'ATD'},
        0x0D: {'str': 'TIM', 'ids': tim_ids},
        0x10: {'str': 'ESF'},
        0x13: {'str': 'MGA'},
        0x21: {'str': 'LOG'},
        0x27: {'str': 'SEC', 'ids': sec_ids},
        0x28: {'str': 'HNR'},
        0xf0: {'str': 'NMEA', 'ids': nmea_ids},
        0xf5: {'str': 'RTCM', 'ids': rtcm_ids},
    }

    def class_id_s(self, m_class, m_id):
        """Return class and ID numbers as a string."""
        s = 'Class: '
        if m_class not in self.classes:
            s += '%#x ID: %#x' % (m_class, m_id)
            return s

        if 'str' in self.classes[m_class]:
            s_class = self.classes[m_class]['str']
            s += '%s(%#x) ' % (s_class, m_class)
        else:
            s += '%#x ' % (m_class)

        if (('ids' in self.classes[m_class] and
             m_id in self.classes[m_class]['ids'] and
             'str' in self.classes[m_class]['ids'][m_id])):
            s_id = self.classes[m_class]['ids'][m_id]['str']
            s += 'ID: %s(%#x)' % (s_id, m_id)
        else:
            s += 'ID: %#x' % (m_id)

        return s

    def decode_msg(self, out):
        """Decode one message and then return number of chars consumed"""

        state = 'BASE'
        consumed = 0

        # decode state machine
        for this_byte in out:
            consumed += 1
            if isinstance(this_byte, str):
                # a character, probably read from a file
                c = ord(this_byte)
            else:
                # a byte, probably read from a serial port
                c = int(this_byte)

            if VERB_RAW <= opts['verbosity']:
                if (ord(' ') <= c) and (ord('~') >= c):
                    # c is printable
                    print("state: %s char %c (%#x)" % (state, chr(c), c))
                else:
                    # c is not printable
                    print("state: %s char %#x" % (state, c))

            if 'BASE' == state:
                # start fresh
                # place to store 'comments'
                comment = ''
                m_class = 0
                m_id = 0
                m_len = 0
                m_raw = bytearray(0)        # class, id, len, payload
                m_payload = bytearray(0)    # just the payload
                m_ck_a = 0
                m_ck_b = 0

                if 0xb5 == c:
                    # got header 1, mu
                    state = 'HEADER1'

                if ord('$') == c:
                    # got $, so NMEA?
                    state = 'NMEA'
                    comment = '$'

                if ord("{") == c:
                    # JSON, treat as comment line
                    state = 'JSON'

                    # start fresh
                    comment = "{"
                    continue

                if ord("#") == c:
                    # comment line
                    state = 'COMMENT'

                    # start fresh
                    comment = "#"
                    continue

                if 0xd3 == c:
                    # RTCM3 Leader 1
                    state = 'RTCM3_1'

                    # start fresh
                    comment = "#"
                    continue

                if (ord('\n') == c) or (ord('\r') == c):
                    # CR or LF, leftovers
                    return 1
                continue

            if state in ('COMMENT', 'JSON'):
                # inside comment
                if ord('\n') == c or ord('\r') == c:
                    # Got newline or linefeed
                    # terminate messages on <CR> or <LF>
                    # Done, got a full message
                    if gps.polystr('{"class":"ERROR"') in comment:
                        # always print gpsd errors
                        print(comment)
                    elif VERB_DECODE <= opts['verbosity']:
                        print(comment)
                    return consumed
                else:
                    comment += chr(c)
                continue

            if 'NMEA' == state:
                # getting NMEA payload
                if (ord('\n') == c) or (ord('\r') == c):
                    # CR or LF, done, got a full message
                    # terminates messages on <CR> or <LF>
                    if VERB_DECODE <= opts['verbosity']:
                        print(comment + '\n')
                    return consumed
                else:
                    comment += chr(c)
                continue

            if 'RTCM3_1' == state:
                # high 6 bits must be zero,
                if 0 != (c & 0xfc):
                    state = 'BASE'
                else:
                    # low 2 bits are MSB of a 10-bit length
                    m_len = c << 8
                    state = 'RTCM3_2'
                    m_raw.extend([c])
                continue

            if 'RTCM3_2' == state:
                # 8 bits are LSB of a 10-bit length
                m_len |= 0xff & c
                # add 3 for checksum
                m_len += 3
                state = 'RTCM3_PAYLOAD'
                m_raw.extend([c])
                continue

            if 'RTCM3_PAYLOAD' == state:
                m_len -= 1
                m_raw.extend([c])
                m_payload.extend([c])
                if 0 == m_len:
                    state = 'BASE'
                    type = m_payload[0] << 4
                    type |= 0x0f & (m_payload[1] >> 4)
                    if VERB_DECODE <= opts['verbosity']:
                        print("RTCM3 packet: type %d\n" % type)
                continue

            if ord('b') == c and 'HEADER1' == state:
                # got header 2
                state = 'HEADER2'
                continue

            if 'HEADER2' == state:
                # got class
                state = 'CLASS'
                m_class = c
                m_raw.extend([c])
                continue

            if 'CLASS' == state:
                # got ID
                state = 'ID'
                m_id = c
                m_raw.extend([c])
                continue

            if 'ID' == state:
                # got first length
                state = 'LEN1'
                m_len = c
                m_raw.extend([c])
                continue

            if 'LEN1' == state:
                # got second length
                m_raw.extend([c])
                m_len += 256 * c
                if 0 == m_len:
                    # no payload
                    state = 'CSUM1'
                else:
                    state = 'PAYLOAD'
                continue

            if 'PAYLOAD' == state:
                # getting payload
                m_raw.extend([c])
                m_payload.extend([c])
                if len(m_payload) == m_len:
                    state = 'CSUM1'
                continue

            if 'CSUM1' == state:
                # got ck_a
                state = 'CSUM2'
                m_ck_a = c
                continue

            if 'CSUM2' == state:
                # ck_b
                state = 'BASE'
                m_ck_b = c
                # check checksum
                chk = self.checksum(m_raw, len(m_raw))
                if (chk[0] != m_ck_a) or (chk[1] != m_ck_b):
                    sys.stderr.write("%s: ERROR checksum failed,"
                                     "was (%d,%d) s/b (%d, %d)\n" %
                                     (PROG_NAME, m_ck_a, m_ck_b,
                                      chk[0], chk[1]))

                s_payload = ''.join('{:02x} '.format(x) for x in m_payload)
                x_payload = ','.join(['%02x' % x for x in m_payload])

                if m_class in self.classes:
                    this_class = self.classes[m_class]
                    if 'ids' in this_class:
                        if m_id in this_class['ids']:
                            if 'dec' in this_class['ids'][m_id]:
                                dec = this_class['ids'][m_id]['dec']
                                s_payload = this_class['ids'][m_id]['name']
                                s_payload += ':\n'
                                s_payload += dec(self, m_payload)
                            else:
                                s_payload = ("%s, len %#x, raw %s" %
                                             (self.class_id_s(m_class, m_id),
                                              m_len, x_payload))

                if VERB_INFO <= opts['verbosity']:
                    print("%s, len: %#x" %
                          (self.class_id_s(m_class, m_id), m_len))
                    print("payload: %s" % x_payload)
                print("%s\n" % s_payload)
                return consumed

            # give up
            state = 'BASE'

        # fell out of loop, no more chars to look at
        return 0

    def checksum(self, msg, m_len):
        """Calculate u-blox message checksum"""
        # the checksum is calculated over the Message, starting and including
        # the CLASS field, up until, but excluding, the Checksum Field:

        ck_a = 0
        ck_b = 0
        for c in msg[0:m_len]:
            ck_a += c
            ck_b += ck_a

        return [ck_a & 0xff, ck_b & 0xff]

    def make_pkt(self, m_class, m_id, m_data):
        """Make a message packet"""
        # always little endian, leader, class, id, length
        m_len = len(m_data)

        # build core message
        msg = bytearray(m_len + 6)
        struct.pack_into('<BBH', msg, 0, m_class, m_id, m_len)

        # copy payload into message buffer
        i = 0
        while i < m_len:
            msg[i + 4] = m_data[i]
            i += 1

        # add checksum
        chk = self.checksum(msg, m_len + 4)
        m_chk = bytearray(2)
        struct.pack_into('<BB', m_chk, 0, chk[0], chk[1])

        header = b"\xb5\x62"
        return header + msg[:m_len + 4] + m_chk

    def gps_send(self, m_class, m_id, m_data):
        """Build, and send, a message to GPS"""
        m_all = self.make_pkt(m_class, m_id, m_data)
        self.gps_send_raw(m_all)

    def gps_send_raw(self, m_all):
        """Send a raw message to GPS"""
        if not opts['read_only']:
            io_handle.ser.write(m_all)
            if VERB_QUIET < opts['verbosity']:
                sys.stdout.write("sent:\n")
                if VERB_INFO < opts['verbosity']:
                    sys.stdout.write(gps.polystr(binascii.hexlify(m_all)))
                    sys.stdout.write("\n")
                self.decode_msg(m_all)
                sys.stdout.flush()

    def send_able_beidou(self, able):
        """dis/enable BeiDou"""
        # Two frequency GPS use BeiDou or GLONASS
        # disable, then enable
        gps_model.send_cfg_gnss1(3, able)

    def send_able_binary(self, able):
        """dis/enable basic binary messages"""

        rate = 1 if able else 0

        # UBX-NAV-DOP
        m_data = bytearray([0x01, 0x04, rate])
        gps_model.gps_send(6, 1, m_data)

        if 15 > opts['protver']:
            # UBX-NAV-SOL is ECEF. deprecated in protver 14, gone in protver 27
            m_data = bytearray([0x01, 0x06, rate])
            gps_model.gps_send(6, 1, m_data)
        else:
            # UBX-NAV-PVT
            m_data = bytearray([0x01, 0x07, rate])
            gps_model.gps_send(6, 1, m_data)

        # UBX-NAV-TIMEGPS
        # Note: UTC may, or may not be UBX-NAV-TIMEGPS.
        #       depending on UBX-CFG-NAV5 utcStandard
        # Note: We use TIMEGPS to get the leapS
        m_data = bytearray([0x01, 0x20, rate])
        gps_model.gps_send(6, 1, m_data)

        # no point doing UBX-NAV-SBAS and UBX-NAV-SVINFO
        # faster than every 10 seconds
        if rate:
            rate_s = 10
        else:
            rate_s = 0

        if 27 > opts['protver']:
            # UBX-NAV-SBAS, gone in protver 27
            m_data = bytearray([0x01, 0x32, rate_s])
            gps_model.gps_send(6, 1, m_data)

        # get Satellite Information
        if 15 > opts['protver']:
            # UBX-NAV-SVINFO - deprecated in protver 15, gone in 27
            m_data = bytearray([0x01, 0x30, rate_s])
            gps_model.gps_send(6, 1, m_data)

            # UBX-NAV-SAT turn it off, if we can
            m_data = bytearray([0x01, 0x35, 0])
            gps_model.gps_send(6, 1, m_data)
        else:
            # use UBX-NAV-SAT for protver 15 and up
            m_data = bytearray([0x01, 0x35, rate_s])
            gps_model.gps_send(6, 1, m_data)

            if 27 > opts['protver']:
                # UBX-NAV-SVINFO turn it off, if we can
                m_data = bytearray([0x01, 0x30, 0])
                gps_model.gps_send(6, 1, m_data)

        if 18 <= opts['protver']:
            # first in u-blox 8
            # UBX-NAV-EOE, end of epoch. Good cycle ender
            m_data = bytearray([0x01, 0x61, rate])
            gps_model.gps_send(6, 1, m_data)

    def send_able_ecef(self, able):
        """Enable ECEF messages"""
        # set NAV-POSECEF rate
        gps_model.send_cfg_msg(1, 1, able)
        # set NAV-VELECEF rate
        gps_model.send_cfg_msg(1, 0x11, able)

    def send_able_gps(self, able):
        """dis/enable GPS/QZSS"""
        # GPS and QZSS both on, or both off, together
        # GPS
        gps_model.send_cfg_gnss1(0, able)
        # QZSS
        gps_model.send_cfg_gnss1(5, able)

    def send_able_galileo(self, able):
        """dis/enable GALILEO"""
        gps_model.send_cfg_gnss1(2, able)

    def send_able_glonass(self, able):
        """dis/enable GLONASS"""
        # Two frequency GPS use BeiDou or GLONASS
        # disable, then enable
        gps_model.send_cfg_gnss1(6, able)

    def send_able_nmea(self, able):
        """dis/enable basic NMEA messages"""

        rate = 1 if able else 0

        # xxGBS
        m_data = bytearray([0xf0, 0x09, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxGGA
        m_data = bytearray([0xf0, 0x00, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxGGL
        m_data = bytearray([0xf0, 0x01, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxGSA
        m_data = bytearray([0xf0, 0x02, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxGST
        m_data = bytearray([0xf0, 0x07, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxGSV
        m_data = bytearray([0xf0, 0x03, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxRMC
        m_data = bytearray([0xf0, 0x04, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxVTG
        m_data = bytearray([0xf0, 0x05, rate])
        gps_model.gps_send(6, 1, m_data)

        # xxZDA
        m_data = bytearray([0xf0, 0x08, rate])
        gps_model.gps_send(6, 1, m_data)

    def send_able_rawx(self, able):
        """dis/enable UBX-RXM-RAW/RAWXX"""

        rate = 1 if able else 0
        if 15 > opts['protver']:
            # u-blox 7 or earlier, use RAW
            sid = 0x10
        else:
            # u-blox 8 or later, use RAWX
            sid = 0x15
        m_data = bytearray([0x2, sid, rate])
        gps_model.gps_send(6, 1, m_data)

    def send_able_pps(self, able):
        """dis/enable PPS, using UBX-CFG-TP5"""

        m_data = bytearray(32)
        m_data[0] = 0         # tpIdx
        m_data[1] = 1         # version
        m_data[2] = 0         # reserved
        m_data[3] = 0         # reserved
        m_data[4] = 2         # antCableDelay
        m_data[5] = 0         # antCableDelay
        m_data[6] = 0         # rfGroupDelay
        m_data[7] = 0         # rfGroupDelay
        m_data[8] = 0x40      # freqPeriod
        m_data[9] = 0x42      # freqPeriod
        m_data[10] = 0x0f     # freqPeriod
        m_data[11] = 0        # freqPeriod
        m_data[12] = 0x40     # freqPeriodLock
        m_data[13] = 0x42     # freqPeriodLock
        m_data[14] = 0x0f     # freqPeriodLock
        m_data[15] = 0        # freqPeriodLock
        m_data[16] = 0        # pulseLenRatio
        m_data[17] = 0        # pulseLenRatio
        m_data[18] = 0        # pulseLenRatio
        m_data[19] = 0        # pulseLenRatio
        m_data[20] = 0xa0     # pulseLenRatioLock
        m_data[21] = 0x86     # pulseLenRatioLock
        m_data[22] = 0x1      # pulseLenRatioLock
        m_data[23] = 0        # pulseLenRatioLock
        m_data[24] = 0        # userConfigDelay
        m_data[25] = 0        # userConfigDelay
        m_data[26] = 0        # userConfigDelay
        m_data[27] = 0        # userConfigDelay
        m_data[28] = 0x77     # flags
        m_data[29] = 0        # flags
        m_data[30] = 0        # flags
        m_data[31] = 0        # flags

        gps_model.gps_send(6, 0x31, m_data)

    def send_able_sbas(self, able):
        """dis/enable SBAS"""
        gps_model.send_cfg_gnss1(1, able)

    def send_able_sfrbx(self, able):
        """dis/enable UBX-RXM-SFRB/SFRBX"""

        rate = 1 if able else 0
        if 15 > opts['protver']:
            # u-blox 7 or earlier, use SFRB
            sid = 0x11
        else:
            # u-blox 8 or later, use SFRBX
            sid = 0x13
        m_data = bytearray([0x2, sid, rate])
        gps_model.gps_send(6, 1, m_data)

    def send_able_tmode2(self, able):
        """SURVEYIN, UBX-CFG-TMODE2, set time mode 2 config"""

        m_data = bytearray(28)
        if able:
            # enable survey-in
            m_data[0] = 1

        # on a NEO-M8T, with good antenna
        # five minutes, gets about 1 m
        # ten minutes, gets about 0.9 m
        # twenty minutes, gets about 0.7 m
        # one hour, gets about 0.5 m
        # twelve hours, gets about 0.14 m

        # Survey-in minimum duration seconds
        seconds = 300
        m_data[20] = seconds & 0x0ff
        seconds >>= 8
        m_data[21] = seconds & 0x0ff
        seconds >>= 8
        m_data[22] = seconds & 0x0ff
        seconds >>= 8
        m_data[23] = seconds & 0x0ff

        # Survey-in position accuracy limit in mm
        # make it big, so the duration decides when to end survey
        mmeters = 50000
        m_data[24] = mmeters & 0x0ff
        mmeters >>= 8
        m_data[25] = mmeters & 0x0ff
        mmeters >>= 8
        m_data[26] = seconds & 0x0ff
        seconds >>= 8
        m_data[27] = mmeters & 0x0ff
        gps_model.gps_send(6, 0x3d, m_data)

    def send_able_tp(self, able):
        """dis/enable UBX-TIM-TP Time Pulse"""
        rate = 1 if able else 0
        m_data = bytearray([0xd, 0x1, rate])
        gps_model.gps_send(6, 1, m_data)

    def send_cfg_cfg(self, save_clear):
        """UBX-CFG-CFG, save config"""

        # Save: save_clear = 0
        # Clear: save_clear = 1

        # basic configs always available to change:
        # ioPort, msgConf, infMsg, navConf, rxmConf
        cfg1 = 0x1f
        # senConf, rinvConf, antConf, logConf
        cfg2 = 0x0f

        m_data = bytearray(13)

        # clear
        # as of protver 27, any bit in clearMask clears all
        if 0 == save_clear:
            # saving
            m_data[0] = 0
            m_data[1] = 0
        else:
            # clearing
            m_data[0] = cfg1
            m_data[1] = cfg2
        m_data[2] = 0       #
        m_data[3] = 0       #

        # save
        # as of protver 27, any bit in saveMask saves all
        if 0 == save_clear:
            # saving
            m_data[4] = cfg1
            m_data[5] = cfg2
        else:
            # clearing
            m_data[4] = 0
            m_data[5] = 0
        m_data[6] = 0       #
        m_data[7] = 0       #

        # load
        # as of protver 27, any bit in loadMask loads all
        if False and 0 == save_clear:
            # saving
            m_data[8] = 0
            m_data[9] = 0
        else:
            # clearing, load it to save a reboot
            m_data[8] = cfg1
            m_data[9] = cfg2
        m_data[10] = 0      #
        m_data[11] = 0      #

        # deviceMask, where to save it, try all options
        m_data[12] = 0x17      # devBBR, devFLASH devEEPROM, devSpiFlash

        gps_model.gps_send(6, 0x9, m_data)

    def send_cfg_gnss1(self, gnssId, enable):
        """UBX-CFG-GNSS, set GNSS config"""
        m_data = bytearray(12)
        m_data[0] = 0       # version 0, msgVer
        m_data[1] = 0       # read only, numTrkChHw
        m_data[2] = 0xFF    # read only, numTrkChUse
        m_data[3] = 1       # 1 block follows
        # block 1
        m_data[4] = gnssId  # gnssId
        if 0 == gnssId:
            # GPS
            m_data[5] = 8   # resTrkCh
            m_data[6] = 16  # maxTrkCh
        if 1 == gnssId:
            # SBAS
            m_data[5] = 1   # resTrkCh
            m_data[6] = 3   # maxTrkCh
        if 2 == gnssId:
            # GALILEO
            m_data[5] = 4   # resTrkCh
            m_data[6] = 8   # maxTrkCh
        if 3 == gnssId:
            # BeiDou
            m_data[5] = 2   # resTrkCh
            m_data[6] = 16  # maxTrkCh
        if 4 == gnssId:
            # IMES
            m_data[5] = 0   # resTrkCh
            m_data[6] = 8   # maxTrkCh
        if 5 == gnssId:
            # QZSS
            m_data[5] = 0   # resTrkCh
            m_data[6] = 3   # maxTrkCh
        if 6 == gnssId:
            # GLONASS
            m_data[5] = 8   # resTrkCh
            m_data[6] = 14  # maxTrkCh
        m_data[7] = 0       # reserved1
        m_data[8] = enable  # flags
        m_data[9] = 0       # flagflags, unused
        if 5 == gnssId:
            # QZSS
            m_data[10] = 5      # flags E1OS, L1SAIF
        else:
            m_data[10] = 1      # flags E1OS
        m_data[11] = 1      # flags, unused
        gps_model.gps_send(6, 0x3e, m_data)

    def poll_cfg_inf(self):
        """UBX-CFG-INF, poll"""

        m_data = bytearray(1)
        m_data[0] = 0        # UBX
        gps_model.gps_send(6, 0x02, m_data)

        m_data[0] = 1        # NMEA
        gps_model.gps_send(6, 0x02, m_data)

    def send_cfg_nav5_model(self):
        """UBX-CFG-NAV5, set dynamic platform model"""

        m_data = bytearray(36)
        m_data[0] = 1        # just setting dynamic model
        m_data[1] = 0        # just setting dynamic model
        m_data[2] = opts["mode"]

        gps_model.gps_send(6, 0x24, m_data)

    def send_cfg_msg(self, m_class, m_id, rate=None):
        """UBX-CFG-MSG, poll, or set, message rates decode"""
        m_data = bytearray(2)
        m_data[0] = m_class
        m_data[1] = m_id
        if rate is not None:
            m_data.extend([rate])
        gps_model.gps_send(6, 1, m_data)

    def send_cfg_pms(self):
        """UBX-CFG-PMS, poll/set Power Management Settings"""

        if opts["mode"] is not None:
            m_data = bytearray(8)
            # set powerSetupValue to mode
            m_data[1] = opts["mode"]
            # leave period and onTime zero, which breaks powerSetupValue = 3
        else:
            m_data = bytearray(0)

        gps_model.gps_send(6, 0x86, m_data)

    def send_cfg_prt(self):
        """UBX-CFG-PRT, get I/O Port settings"""
        port = opts['interface']
        if port is None:
            m_data = bytearray()
        else:
            m_data = bytearray([port])
        gps_model.gps_send(6, 0x0, m_data)

    def send_set_speed(self, speed):
        """"UBX-CFG-PRT, set port"""
        port = opts['interface']
        # FIXME!  Determine and use current port as default
        if port is None:
            port = 1  # Default to port 1 (UART/UART_1)
        if port not in set([1, 2]):
            sys.stderr.write('%s: Invalid UART port - %d\n' %
                             (PROG_NAME, port))
            sys.exit(2)

        # FIXME!  Poll current masks, then adjust speed
        m_data = bytearray(20)
        m_data[0] = port
        m_data[4] = 0xc0          # 8N1
        m_data[5] = 0x8           # 8N1

        m_data[8] = speed & 0xff
        m_data[9] = (speed >> 8) & 0xff
        m_data[10] = (speed >> 16) & 0xff
        m_data[11] = (speed >> 24) & 0xff

        m_data[12] = 3             # in, ubx and nmea
        m_data[14] = 3             # out, ubx and nmea
        gps_model.gps_send(6, 0, m_data)

    def send_cfg_rst(self, reset_type):
        """UBX-CFG-RST, reset"""
        # always do a hardware reset
        # if on USB, this will disconnect and reconnect, giving you
        # a new tty.
        m_data = bytearray(4)
        m_data[0] = reset_type & 0xff
        m_data[1] = (reset_type >> 8) & 0xff
        gps_model.gps_send(6, 0x4, m_data)

    def send_cfg_tp5(self):
        """UBX-CFG-TP5, get time0 decodes, timepulse 0 and 1"""
        m_data = bytearray(0)
        gps_model.gps_send(6, 0x31, m_data)
        # and timepulse 1
        m_data = bytearray(1)
        m_data[0] = 1
        gps_model.gps_send(6, 0x31, m_data)

    def send_mon_comms(self):
        """UBX-MON-COMMS Comm port information"""
        m_data = bytearray(0)
        gps_model.gps_send(0x0a, 0x36, m_data)

    def send_cfg_valdel(self, key):
        """UBX-CFG-VALDEL, delete config items by key"""
        m_data = bytearray(4)
        m_data[0] = 0       # version, 0 = transactionless, 1 = transaction
        m_data[1] = 6       # 2 = BBR, 4 = flash
        # can not delete RAM layer!
        # so options stay set until reset!

        for key in keys:
            k_data = bytearray(4)
            k_data[0] = (key) & 0xff
            k_data[1] = (key >> 8) & 0xff
            k_data[2] = (key >> 16) & 0xff
            k_data[3] = (key >> 24) & 0xff
            m_data.extend(k_data)
        gps_model.gps_send(0x06, 0x8c, m_data)

    def send_cfg_valget(self, keys):
        """UBX-CFG-VALGET, get config items by key"""
        m_data = bytearray(4)
        m_data[0] = 0      # version, 0 = request, 1 = answer
        m_data[1] = 0      # RAM layer
        for key in keys:
            k_data = bytearray(4)
            k_data[0] = (key) & 0xff
            k_data[1] = (key >> 8) & 0xff
            k_data[2] = (key >> 16) & 0xff
            k_data[3] = (key >> 24) & 0xff
            m_data.extend(k_data)
        gps_model.gps_send(0x06, 0x8b, m_data)

    def send_cfg_valset(self, nvs):
        """UBX-CFG-VALSET, set config items by key/val pairs"""

        m_data = bytearray(4)
        m_data[0] = 0      # version, 0 = request, 1 = transaction
        m_data[1] = 0x7    # RAM layer, 1=RAM, 2=BBR, 4=Flash

        for nv in nvs:
            size = 4
            nv_split = nv.split(',')
            name = nv_split[0]
            val = nv_split[1]

            item = gps_model.cfg_by_name(name)
            key = item[1]
            val_type = item[2]

            cfg_type = self.item_to_type(item)

            size = 4 + cfg_type[0]
            frmat = cfg_type[1]
            flavor = cfg_type[2]
            if 'u' == flavor:
                val1 = int(val)
            elif 'i' == flavor:
                val1 = int(val)
            elif 'f' == flavor:
                val1 = float(val)

            kv_data = bytearray(size)
            kv_data[0] = (key) & 0xff
            kv_data[1] = (key >> 8) & 0xff
            kv_data[2] = (key >> 16) & 0xff
            kv_data[3] = (key >> 24) & 0xff

            struct.pack_into(frmat, kv_data, 4, val1)
            m_data.extend(kv_data)
        gps_model.gps_send(0x06, 0x8a, m_data)

    def send_poll(self, m_data):
        """generic send poll request"""
        gps_model.gps_send(m_data[0], m_data[1], m_data[2:])

    able_commands = {
        # en/dis able BeiDou
        "BEIDOU": {"command": send_able_beidou,
                   "help": "BeiDou"},
        # en/dis able basic binary messages
        "BINARY": {"command": send_able_binary,
                   "help": "basic binary messages"},
        # en/dis able ECEF
        "ECEF": {"command": send_able_ecef,
                 "help": "ECEF"},
        # en/dis able GPS
        "GPS": {"command": send_able_gps,
                "help": "GPS and QZSS"},
        # en/dis able GALILEO
        "GALILEO": {"command": send_able_galileo,
                    "help": "GALILEO"},
        # en/dis able GLONASS
        "GLONASS": {"command": send_able_glonass,
                    "help": "GLONASS"},
        # en/dis able basic NMEA messages
        "NMEA": {"command": send_able_nmea,
                 "help": "basic NMEA messages"},
        # en/dis able RAW/RAWX
        "RAWX": {"command": send_able_rawx,
                 "help": "RAW/RAWX measurements"},
        # en/dis able PPS
        "PPS": {"command": send_able_pps,
                "help": "PPS on TIMPULSE"},
        # en/dis able SBAS
        "SBAS": {"command": send_able_sbas,
                 "help": "SBAS"},
        # en/dis able SFRB/SFRBX
        "SFRBX": {"command": send_able_sfrbx,
                  "help": "SFRB/SFRBX subframes"},
        # en/dis able TP time pulse message
        "TP": {"command": send_able_tp,
               "help": "TP Time Pulse message"},
        # en/dis able TMODE2 Survey-in
        "SURVEYIN": {"command": send_able_tmode2,
                     "help": "Survey-in mode with TMODE2"},
    }
    commands = {
        # UBX-CFG-ANT
        "CFG-ANT": {"command": send_poll, "opt": [0x06, 0x13],
                    "help": "poll UBX-CFG-ANT antenna config"},
        # UBX-CFG-GNSS
        "CFG-GNSS": {"command": send_poll, "opt": [0x06, 0x3e],
                     "help": "poll UBX-CFG-GNSS GNSS config"},
        # UBX-CFG-INF
        "CFG-INF": {"command": poll_cfg_inf,
                    "help": "poll UBX-CFG-INF Information Message "
                            "Configuration"},
        # UBX-CFG-NAV5
        "CFG-NAV5": {"command": send_poll, "opt": [0x06, 0x24],
                     "help": "poll UBX-CFG-NAV5 Nav Engines settings"},
        # UBX-CFG-NAVX5
        "CFG-NAVX5": {"command": send_poll, "opt": [0x06, 0x23],
                      "help": "poll UBX-CFG-NAVX5 Nav Expert Settings"},
        # UBX-CFG-NMEA
        "CFG-NMEA": {"command": send_poll, "opt": [0x06, 0x17],
                     "help": "poll UBX-CFG-NMEA Extended NMEA protocol "
                             "configuration V1"},
        # UBX-CFG-ODO
        "CFG-ODO": {"command": send_poll, "opt": [0x06, 0x1e],
                    "help": "poll UBX-CFG-ODO Odometer, Low-speed COG "
                            "Engine Settings"},
        # UBX-CFG-PMS
        "CFG-PMS": {"command": send_poll, "opt": [0x06, 0x86],
                    "help": "poll UBX-CFG-PMS power management settings"},
        # UBX-CFG-PRT
        "CFG-PRT": {"command": send_cfg_prt,
                    "help": "poll UBX-CFG-PRT I/O port settings"},
        # UBX-CFG-RATE
        "CFG-RATE": {"command": send_poll, "opt": [0x06, 0x08],
                     "help": "poll UBX-CFG-RATE Navigation/Measurement "
                             "Rate Settings"},
        # UBX-CFG-RINV
        "CFG-RINV": {"command": send_poll, "opt": [0x06, 0x34],
                     "help": "poll UBX-CFG-RINV Contents of Remote Inventory"},
        # UBX-CFG-SBAS
        "CFG-SBAS": {"command": send_poll, "opt": [0x06, 0x16],
                     "help": "poll UBX-CFG-SBAS SBAS settings"},
        # UBX-CFG-SMGR
        "CFG-SMGR": {"command": send_poll, "opt": [0x06, 0x62],
                     "help": "poll UBX-CFG-SMGR Synchronization manager "
                     " configuration"},
        # UBX-CFG-TMODE2
        "CFG-TMODE2": {"command": send_poll, "opt": [0x06, 0x3d],
                       "help": "poll UBX-CFG-TMODE2 time mode 2 config"},
        # UBX-CFG-TP5
        "CFG-TP5": {"command": send_cfg_tp5,
                    "help": "poll UBX-TIM-TP5 time pulse decodes"},
        # UBX-CFG-USB
        "CFG-USB": {"command": send_poll, "opt": [0x06, 0x1b],
                    "help": "poll UBX-CFG-USB USB config"},
        # UBX-CFG-RST
        "COLDBOOT": {"command": send_cfg_rst,
                     "help": "UBS-CFG-RST coldboot the GPS",
                     "opt": 0xfff},
        # UBX-CFG-RST
        "HOTBOOT": {"command": send_cfg_rst,
                    "help": "UBX-CFG-RST hotboot the GPS",
                    "opt": 0},
        # UBX-CFG-NAV5
        "MODEL": {"command": send_cfg_nav5_model,
                  "help": "set UBX-CFG-NAV5 Dynamic Platform Model"},
        # UBX-MON-COMMS
        "MON-COMMS": {"command": send_mon_comms,
                      "help": "poll UBX-MON-COMMS Comm port "
                      " information (27+)"},
        # UBX-MON-GNSS
        "MON-GNSS": {"command": send_poll, "opt": [0x0a, 0x28],
                     "help": "poll UBX-MON-GNSS major GNSS selection"},
        # UBX-MON-HW
        "MON-HW": {"command": send_poll, "opt": [0x0a, 0x09],
                   "help": "poll UBX-MON-HW Hardware Status"},
        # UBX-MON-HW2
        "MON-HW2": {"command": send_poll, "opt": [0x0a, 0x0b],
                    "help": "poll UBX-MON-HW2 Exended Hardware Status"},
        # UBX-MON-HW3
        "MON-HW3": {"command": send_poll, "opt": [0x0a, 0x37],
                    "help": "poll UBX-MON-HW3 HW I/O pin infromation"},
        # UBX-MON-IO
        "MON-IO": {"command": send_poll, "opt": [0x0a, 0x02],
                   "help": "poll UBX-MON-IO I/O Subsystem Status"},
        # UBX-MON-MSGPP
        "MON-MSGPP": {"command": send_poll, "opt": [0x0a, 0x06],
                      "help": "poll UBX-MON-MSGPP Message Parese and "
                              "Process Status"},
        # UBX-MON-PATCH
        "MON-PATCH": {"command": send_poll, "opt": [0x0a, 0x27],
                      "help": "poll UBX-MON-PATCH Info on Installed Patches"},
        # UBX-MON-RF
        "MON-RF": {"command": send_poll, "opt": [0x0a, 0x38],
                   "help": "poll UBX-MON-RF RF Information"},
        # UBX-MON-RXBUF
        "MON-RXBUF": {"command": send_poll, "opt": [0x0a, 0x07],
                      "help": "poll UBX-MON-RXBUF Receiver Buffer Status"},
        # UBX-MON-SMGR
        "MON-SMGR": {"command": send_poll, "opt": [0x0a, 0x2e],
                     "help": "poll UBX-MON-SMGR Synchronization manager "
                     "configuration"},
        # UBX-MON-TXBUF
        "MON-TXBUF": {"command": send_poll, "opt": [0x0a, 0x08],
                      "help": "poll UBX-MON-TXBUF Transmitter Buffer Status"},
        # UBX-MON-VER
        "MON-VER": {"command": send_poll, "opt": [0x0a, 0x04],
                    "help": "poll UBX-MON-VER GPS version"},
        # UBX-NAV-CLOCK
        "NAV-CLOCK": {"command": send_poll, "opt": [0x01, 0x22],
                      "help": "poll UBX-NAV-CLOCK Clock Solution"},
        # UBX-NAV-DGPS
        "NAV-DGPS": {"command": send_poll, "opt": [0x01, 0x31],
                     "help": "poll UBX-NAV-DGPS DGPS Data Used for NAV"},
        # UBX-NAV-DOP
        "NAV-DOP": {"command": send_poll, "opt": [0x01, 0x04],
                    "help": "poll UBX-NAV-DOP Dilution of Precision"},
        # UBX-NAV-GEOFENCE
        "NAV-GEOFENCE": {"command": send_poll, "opt": [0x01, 0x39],
                         "help": "poll UBX-NAV-GEOFENCE Geofence status"},
        # UBX-NAV-HPPOSECEF
        "NAV-HPPOSECEF": {"command": send_poll, "opt": [0x01, 0x13],
                          "help": "poll UBX-NAV-HPPOSECEF ECEF position"},
        # UBX-NAV-HPPOSLLH
        "NAV-HPPOSLLH": {"command": send_poll, "opt": [0x01, 0x14],
                         "help": "poll UBX-NAV-HPPOSECEF LLH position"},
        # UBX-NAV-POSECEF
        "NAV-POSECEF": {"command": send_poll, "opt": [0x01, 0x01],
                        "help": "poll UBX-NAV-POSECEF ECEF position"},
        # UBX-NAV-POSLLH
        "NAV-POSLLH": {"command": send_poll, "opt": [0x01, 0x02],
                       "help": "poll UBX-NAV-POSLLH LLH position"},
        # UBX-NAV-PVT
        "NAV-PVT": {"command": send_poll, "opt": [0x01, 0x07],
                    "help": "poll UBX-NAV-PVT Navigation Position Velocity "
                    "Time Solution"},
        # UBX-NAV-SAT
        "NAV-SAT": {"command": send_poll, "opt": [0x01, 0x35],
                    "help": "poll UBX-NAV-SAT Satellite Information"},
        # UBX-NAV-SBAS
        "NAV-SBAS": {"command": send_poll, "opt": [0x01, 0x32],
                     "help": "poll UBX-NAV-SBAS SBAS Status Data"},
        # UBX-NAV-SIG
        "NAV-SIG": {"command": send_poll, "opt": [0x01, 0x43],
                    "help": "poll UBX-NAV-SIG Signal Information"},
        # UBX-NAV-SOL
        "NAV-SOL": {"command": send_poll, "opt": [0x01, 0x06],
                    "help": "poll UBX-NAV-SOL Navigation Solution "
                    "Information"},
        # UBX-NAV-STATUS
        "NAV-STATUS": {"command": send_poll, "opt": [0x01, 0x03],
                       "help": "poll UBX-NAV-STATUS Receiver Nav Status"},
        # UBX-NAV-TIMEBDS
        "NAV-TIMEBDS": {"command": send_poll, "opt": [0x01, 0x24],
                        "help": "poll UBX-NAV-TIMEBDS BDS Time Solution"},
        # UBX-NAV-TIMEGAL
        "NAV-TIMEGAL": {"command": send_poll, "opt": [0x01, 0x25],
                        "help": "poll UBX-NAV-TIMEGAL Galileo Time Solution"},
        # UBX-NAV-TIMEGLO
        "NAV-TIMEGLO": {"command": send_poll, "opt": [0x01, 0x23],
                        "help": "poll UBX-NAV-TIMEGLO GLO Time Solution"},
        # UBX-NAV-TIMEGPS
        "NAV-TIMEGPS": {"command": send_poll, "opt": [0x01, 0x20],
                        "help": "poll UBX-NAV-TIMEGPS GPS Time Solution"},
        # UBX-NAV-TIMELS
        "NAV-TIMELS": {"command": send_poll, "opt": [0x01, 0x26],
                       "help": "poll UBX-NAV-TIMELS Leap Second Info"},
        # UBX-NAV-TIMEUTC
        "NAV-TIMEUTC": {"command": send_poll, "opt": [0x01, 0x21],
                        "help": "poll UBX-NAV-TIMEUTC UTC Time Solution"},
        # UBX-NAV-VELECEF
        "NAV-VELECEF": {"command": send_poll, "opt": [0x01, 0x11],
                        "help": "poll UBX-NAV-VELECEF ECEF velocity"},
        # UBX-NAV-VELNED
        "NAV-VELNED": {"command": send_poll, "opt": [0x01, 0x12],
                       "help": "poll UBX-NAV-VELNED NED velocity"},
        # UBX-CFG-PMS
        "PMS": {"command": send_cfg_pms,
                "help": "set UBX-CFG-PMS power management settings"},
        # UBX-RXM-RAWX
        "RXM-RAWX": {"command": send_poll, "opt": [0x02, 0x15],
                     "help": "poll UBX-RXM-RAWX raw measurement data"},
        # UBX-CFG-CFG
        "RESET": {"command": send_cfg_cfg,
                  "help": "UBX-CFG-CFG reset config to defaults",
                  "opt": 1},
        # UBX-CFG-CFG
        "SAVE": {"command": send_cfg_cfg,
                 "help": "UBX-CFG-CFG save current config",
                 "opt": 0},
        # UBX-CFG-SBAS
        "SEC-UNIQID": {"command": send_poll, "opt": [0x27, 0x03],
                       "help": "poll UBX-SEC-UNIQID Unique chip ID"},
        # UBX-TIM-SVIN
        "TIM-SVIN": {"command": send_poll, "opt": [0x0d, 0x04],
                     "help": "poll UBX-TIM-SVIN survey in data"},
        # UBX-TIM-TM2
        "TIM-TM2": {"command": send_poll, "opt": [0x0d, 0x03],
                    "help": "poll UBX-TIM-TM2 time mark data"},
        # UBX-TIM-TP
        "TIM-TP": {"command": send_poll, "opt": [0x0d, 0x01],
                   "help": "poll UBX-TIM-TP time pulse timedata"},
        # UBX-CFG-RST
        "WARMBOOT": {"command": send_cfg_rst,
                     "help": "UBX-CFG-RST warmboot the GPS",
                     "opt": 1},
    }
    # end class ubx


class gps_io(object):
    """All the GPS I/O in one place"

    Three types of GPS I/O
    1. read only from a file
    2. read/write through a device
    3. read only from a gpsd instance
    """

    out = b''
    ser = None
    input_is_device = False

    def __init__(self):
        """Initialize class"""

        Serial = serial
        Serial_v3 = Serial and Serial.VERSION.split('.')[0] >= '3'
        # buffer to hold read data
        self.out = b''

        # open the input: device, file, or gpsd
        if opts['input_file_name'] is not None:
            # check if input file is a file or device
            try:
                mode = os.stat(opts['input_file_name']).st_mode
            except OSError:
                sys.stderr.write('%s: failed to open input file %s\n' %
                                 (PROG_NAME, opts['input_file_name']))
                sys.exit(1)

            if stat.S_ISCHR(mode):
                # character device, need not be read only
                self.input_is_device = True

            if ((opts['disable'] or opts['enable'] or opts['poll'] or
                 opts['oaf_name'])):

                # check that we can write
                if opts['read_only']:
                    sys.stderr.write('%s: read-only mode, '
                                     'can not send commands\n' % PROG_NAME)
                    sys.exit(1)
                if self.input_is_device is False:
                    sys.stderr.write('%s: input is plain file, '
                                     'can not send commands\n' % PROG_NAME)
                    sys.exit(1)

        if opts['target']['server'] is not None:
            # try to open local gpsd
            try:
                self.ser = gps.gpscommon(host=None)
                self.ser.connect(opts['target']['server'],
                                 opts['target']['port'])

                # alias self.ser.write() to self.write_gpsd()
                self.ser.write = self.write_gpsd

                # ask for raw, not rare, data
                data_out = b'?WATCH={'
                if opts['target']['device'] is not None:
                    # add in the requested device
                    data_out += (b'"device":"' + opts['target']['device'] +
                                 b'",')
                data_out += b'"enable":true,"raw":2}\r\n'
                if VERB_RAW <= opts['verbosity']:
                    print("sent: ", data_out)
                self.ser.send(data_out)
            except socket.error as err:
                sys.stderr.write('%s: failed to connect to gpsd %s\n' %
                                 (PROG_NAME, err))
                sys.exit(1)

        elif self.input_is_device:
            # configure the serial connections (the parameters refer to
            # the device you are connecting to)

            # pyserial Ver 3.0+ changes writeTimeout to write_timeout
            # Using the wrong one causes an error
            write_timeout_arg = ('write_timeout'
                                 if Serial_v3 else 'writeTimeout')
            try:
                self.ser = Serial.Serial(
                    baudrate=opts['input_speed'],
                    # 8N1 is UBX default
                    bytesize=Serial.EIGHTBITS,
                    parity=Serial.PARITY_NONE,
                    port=opts['input_file_name'],
                    stopbits=Serial.STOPBITS_ONE,
                    # read timeout
                    timeout=0.05,
                    **{write_timeout_arg: 0.5}
                )
            except AttributeError:
                sys.stderr.write('%s: failed to import pyserial\n' % PROG_NAME)
                sys.exit(2)
            except Serial.serialutil.SerialException:
                # this exception happens on bad serial port device name
                sys.stderr.write('%s: failed to open serial port "%s"\n'
                                 '%s: Your computer has the serial ports:\n' %
                                 (PROG_NAME, opts['input_file_name'],
                                  PROG_NAME))

                # print out list of supported ports
                import serial.tools.list_ports as List_Ports
                ports = List_Ports.comports()
                for port in ports:
                    sys.stderr.write("    %s: %s\n" %
                                     (port.device, port.description))
                sys.exit(1)

            # flush input buffer, discarding all its contents
            # pyserial 3.0+ deprecates flushInput() in favor of
            # reset_input_buffer(), but flushInput() is still present.
            self.ser.flushInput()

        else:
            # Read from a plain file of UBX messages
            try:
                self.ser = open(opts['input_file_name'], 'rb')
            except IOError:
                sys.stderr.write('%s: failed to open input %s\n' %
                                 (PROG_NAME, opts['input_file_name']))
                sys.exit(1)

    def read(self, read_opts):
        """Read from device, until timeout or expected message"""

        # are we expecting a certain message?
        if gps_model.expect_statement_identifier:
            # assume failure, until we see expected message
            ret_code = 1
        else:
            # not expecting anything, so OK if we did not see it.
            ret_code = 0

        try:
            if read_opts['target']['server'] is not None:
                # gpsd input
                start = time.clock()
                while read_opts['input_wait'] > (time.clock() - start):
                    # First priority is to be sure the input buffer is read.
                    # This is to prevent input buffer overuns
                    if 0 < self.ser.waiting():
                        # We have serial input waiting, get it
                        # No timeout possible
                        # RTCM3 JSON can be over 4.4k long, so go big
                        new_out = self.ser.sock.recv(8192)
                        if raw is not None:
                            # save to raw file
                            raw.write(new_out)
                        self.out += new_out

                    consumed = gps_model.decode_msg(self.out)
                    self.out = self.out[consumed:]
                    if ((gps_model.expect_statement_identifier and
                         (gps_model.expect_statement_identifier ==
                          gps_model.last_statement_identifier))):
                        # Got what we were waiting for.  Done?
                        ret_code = 0
                        if not read_opts['input_forced_wait']:
                            # Done
                            break

            elif self.input_is_device:
                # input is a serial device
                start = time.clock()
                while read_opts['input_wait'] > (time.clock() - start):
                    # First priority is to be sure the input buffer is read.
                    # This is to prevent input buffer overuns
                    # pyserial 3.0+ deprecates inWaiting() in favor of
                    # in_waiting, but inWaiting() is still present.
                    if 0 < self.ser.inWaiting():
                        # We have serial input waiting, get it
                        # 1024 is comfortably large, almost always the
                        # Read timeout is what causes ser.read() to return
                        new_out = self.ser.read(1024)
                        if raw is not None:
                            # save to raw file
                            raw.write(new_out)
                        self.out += new_out

                    consumed = gps_model.decode_msg(self.out)
                    self.out = self.out[consumed:]
                    if ((gps_model.expect_statement_identifier and
                         (gps_model.expect_statement_identifier ==
                          gps_model.last_statement_identifier))):
                        # Got what we were waiting for.  Done?
                        ret_code = 0
                        if not read_opts['input_forced_wait']:
                            # Done
                            break
            else:
                # ordinary file, so all read at once
                self.out += self.ser.read()
                if raw is not None:
                    # save to raw file
                    raw.write(self.out)

                while True:
                    consumed = gps_model.decode_msg(self.out)
                    self.out = self.out[consumed:]
                    if 0 >= consumed:
                        break

        except IOError:
            # This happens on a good device name, but gpsd already running.
            # or if USB device unplugged
            sys.stderr.write('%s: failed to read %s\n'
                             '%s: Is gpsd already holding the port?\n'
                             % (PROG_NAME, read_opts['input_file_name'],
                                PROG_NAME))
            return 1

        if 0 < ret_code:
            # did not see the message we were expecting to see
            sys.stderr.write('%s: waited %0.2f seconds for, '
                             'but did not get: %%%s%%\n'
                             % (PROG_NAME, read_opts['input_wait'],
                                gps_model.expect_statement_identifier))
        return ret_code

    def write_gpsd(self, data):
        """write data to gpsd daemon"""

        # HEXDATA_MAX = 512, from gps.h, The max hex digits can write.
        # Input data is binary, converting to hex doubles its size.
        # Limit binary data to length 255, so hex data length less than 510.
        if 255 < len(data):
            sys.stderr.write('%s: trying to send %d bytes, max is 255\n'
                             % (PROG_NAME, len(data)))
            return 1

        if opts['target']['device'] is not None:
            # add in the requested device
            data_out = b'?DEVICE={"path":"' + opts['target']['device'] + b'",'
        else:
            data_out = b'?DEVICE={'

        # Convert binary data to hex and build the message.
        data_out += b'"hexdata":"' + binascii.hexlify(data) + b'"}\r\n'
        if VERB_RAW <= opts['verbosity']:
            print("sent: ", data_out)
        self.ser.send(data_out)
        return 0


# instantiate the GPS class
gps_model = ubx()


def usage():
    """Ouput usage information, and exit"""
    print('usage: %s [-c C] [-f F] [-r] [-p P] [-s S] [-v V]\n'
          '               [-hV?] [-S S]\n'
          '       -c C          send raw command C (cls,id...) to GPS\n'
          '       -d D          disable D\n'
          '       -e E          enable E\n'
          '       -f F          open F as file/device\n'
          '                     default: %s\n'
          '       -g G          get config item G\n'
          '       -h            print help, increase -v for extra help\n'
          '       -i I          interface (port) for UBX-CFG-PRT\n'
          '       -m M          optional mode to -p P\n'
          '       -P P          Protocol version for sending commands\n'
          '                     default: %s\n'
          '       -p P          send a prepackaged query P to GPS\n'
          '       -R R          save raw data from GPS in file R\n'
          '                     default: %s\n'
          '       -r            open file/device read only\n'
          '       -S S          set GPS speed to S\n'
          '       -s S          set port speed to S\n'
          '                     default: %s bps\n'
          '       -w W          wait time W before exiting\n'
          '                     default: %s seconds\n'
          '       -V            print version\n'
          '       -v V          Set verbosity level to V, 0 to 4\n'
          '                     default: %s\n'
          '       -x X          delete config item X\n'
          '       -z Z,z        set config item Z to z\n'
          '       -?            print help\n'
          '\n' %
          (PROG_NAME, opts['input_file_name'],
           opts['protver'], opts['raw_file'],
           opts['input_speed'], opts['input_wait'],
           opts['verbosity'])
          )

    if VERB_DECODE <= opts['verbosity']:
        print('D and E can be one of:')
        for item in sorted(gps_model.able_commands.keys()):
            print("    %-13s %s" %
                  (item, gps_model.able_commands[item]["help"]))

        print('\nP can be one of:')
        for item in sorted(gps_model.commands.keys()):
            print("    %-13s %s" % (item, gps_model.commands[item]["help"]))
        print('\n')
        if VERB_DECODE < opts['verbosity']:
            print('\nConfiguration items for -g, -x and -z can be one of:')
            for item in sorted(gps_model.cfgs):
                print("    %s\n"
                      "        %s" % (item[0], item[5]))
            print('\n')

    print('Options can be placed in the UBXOPTS environment variable.\n'
          'UBXOPTS is processed before the CLI options.')
    sys.exit(0)


if 'UBXOPTS' in os.environ:
    # grab the UBXOPTS environment variable for options
    opts['progopts'] = os.environ['UBXOPTS']
    options = opts['progopts'].split(' ') + sys.argv[1:]
else:
    options = sys.argv[1:]


try:
    (options, arguments) = getopt.getopt(options,
                                         "?c:d:e:f:g:hi:m:rP:p:"
                                         "s:w:v:R:S:Vx:z:")
except getopt.GetoptError as err:
    sys.stderr.write("%s: %s\n"
                     "Try '%s -h' for more information.\n" %
                     (PROG_NAME, str(err), PROG_NAME))
    sys.exit(2)

for (opt, val) in options:
    if opt == '-c':
        opts['command'] = val
    elif opt == '-d':
        opts['disable'] = val
    elif opt == '-e':
        opts['enable'] = val
    elif opt == '-f':
        opts['input_file_name'] = val
    elif opt == '-g':
        opts['get_item'].append(val)
    elif opt == '-h' or opt == '-?':
        opts['help'] = True
    elif opt == '-i':
        valnum = gps_model.port_id_map.get(val.upper())
        opts['interface'] = valnum if valnum is not None else int(val)
    elif opt == '-m':
        opts['mode'] = int(val)
    elif opt == '-P':
        opts['protver'] = int(val)
        if 10 > opts['protver']:
            opts['protver'] = 10
        if 27 < opts['protver']:
            opts['protver'] = 27
    elif opt == '-p':
        opts['poll'] = val
    elif opt == '-r':
        opts['read_only'] = True
    elif opt == '-s':
        try:
            opts['input_speed'] = int(val)
        except ValueError:
            sys.stderr.write('%s: -s invalid speed %s\n' %
                             (PROG_NAME, val))
            sys.exit(1)

        if opts['input_speed'] not in gps_model.speeds:
            sys.stderr.write('%s: -s invalid speed %s\n' %
                             (PROG_NAME, opts['input_speed']))
            sys.exit(1)

    elif opt == '-w':
        opts['input_wait'] = float(val)
    elif opt in '-v':
        opts['verbosity'] = int(val)
    elif opt in '-R':
        # raw log file
        opts['raw_file'] = val
    elif opt in '-S':
        opts['set_speed'] = int(val)
        if opts['set_speed'] not in gps_model.speeds:
            sys.stderr.write('%s: -S invalid speed %s\n' %
                             (PROG_NAME, opts['set_speed']))
            sys.exit(1)

    elif opt == '-V':
        # version
        sys.stderr.write('%s: Version %s\n' % (PROG_NAME, gps_version))
        sys.exit(0)
    elif opt == '-x':
        opts['del_item'].append(val)
    elif opt == '-z':
        opts['set_item'].append(val)

if opts['help']:
        usage()

if opts['input_file_name'] is None:
    # no input file given
    # default to local gpsd
    opts['target']['server'] = "localhost"
    opts['target']['port'] = gps.GPSD_PORT
    opts['target']['device'] = None
    if arguments:
        # server[:port[:device]]
        parts = arguments[0].split(':')
        opts['target']['server'] = parts[0]
        if 1 < len(parts):
            opts['target']['port'] = parts[1]
            if 2 < len(parts):
                opts['target']['device'] = parts[2]

elif arguments:
    sys.stderr.write('%s: Both input file and server specified\n' % PROG_NAME)
    sys.exit(1)

if VERB_PROG <= opts['verbosity']:
    # dump all options
    print('Options:')
    for option in sorted(opts):
        print("   %s: %s" % (option, opts[option]))

# done parsing arguments from environment and CLI

try:
    # raw log file requested?
    raw = None
    if opts['raw_file']:
        try:
            raw = open(opts['raw_file'], 'w')
        except IOError:
            sys.stderr.write('%s: failed to open raw file %s\n' %
                             (PROG_NAME, opts['raw_file']))
            sys.exit(1)

    # create the I/O instance
    io_handle = gps_io()

    sys.stdout.flush()

    if opts['disable'] is not None:
        if VERB_QUIET < opts['verbosity']:
            sys.stderr.write('%s: disable %s\n' % (PROG_NAME, opts['disable']))
        if opts['disable'] in gps_model.able_commands:
            command = gps_model.able_commands[opts['disable']]
            command["command"](gps, 0)
        else:
            sys.stderr.write('%s: disable %s not found\n' %
                             (PROG_NAME, opts['disable']))
            sys.exit(1)

    elif opts['enable'] is not None:
        if VERB_QUIET < opts['verbosity']:
            sys.stderr.write('%s: enable %s\n' % (PROG_NAME, opts['enable']))
        if opts['enable'] in gps_model.able_commands:
            command = gps_model.able_commands[opts['enable']]
            command["command"](gps, 1)
        else:
            sys.stderr.write('%s: enable %s not found\n' %
                             (PROG_NAME, opts['enable']))
            sys.exit(1)

    elif opts['poll'] is not None:
        if VERB_QUIET < opts['verbosity']:
            sys.stderr.write('%s: poll %s\n' % (PROG_NAME, opts['poll']))

        if 'MODEL' == opts["poll"]:
            if opts["mode"] is None:
                opts["mode"] = 0   # default to portable model

        if opts['poll'] in gps_model.commands:
            command = gps_model.commands[opts['poll']]
            if 'opt' in command:
                command["command"](gps, command["opt"])
            else:
                command["command"](gps)
        else:
            sys.stderr.write('%s: poll %s not found\n' %
                             (PROG_NAME, opts['poll']))
            sys.exit(1)

    elif opts['set_speed'] is not None:
        gps_model.send_set_speed(opts['set_speed'])

    elif opts['command'] is not None:
        cmd_list = opts['command'].split(',')
        try:
            cmd_data = [int(v, 16) for v in cmd_list]
        except ValueError:
            badarg = True
        else:
            data_or = reduce(operator.or_, cmd_data)
            badarg = data_or != data_or & 0xFF
        if badarg or len(cmd_list) < 2:
            sys.stderr.write('%s: Argument format (hex bytes) is'
                             ' class,id[,payload...]\n' % PROG_NAME)
            sys.exit(1)
        payload = bytearray(cmd_data[2:])
        if VERB_QUIET < opts['verbosity']:
            sys.stderr.write('%s: command %s\n' % (PROG_NAME, opts['command']))
        gps_model.gps_send(cmd_data[0], cmd_data[1], payload)

    elif opts['del_item']:
        keys = []
        for name in opts['del_item']:
            item = gps_model.cfg_by_name(name)
            if item:
                keys.append(item[1])
            else:
                sys.stderr.write('%s: ERROR: item %s unknown\n' %
                                 (PROG_NAME, opts['del_item']))
                exit(1)
        gps_model.send_cfg_valdel(keys)

    elif opts['get_item']:
        keys = []
        for name in opts['get_item']:
            item = gps_model.cfg_by_name(name)
            if item:
                keys.append(item[1])
            else:
                sys.stderr.write('%s: ERROR: item %s unknown\n' %
                                 (PROG_NAME, name))
                exit(1)
        gps_model.send_cfg_valget(keys)

    elif opts['set_item']:
        nvs = []
        for nv in opts['set_item']:
            (name, val) = nv.split(',')
            item = gps_model.cfg_by_name(name)
            if item:
                nvs.append(nv)
            else:
                sys.stderr.write('%s: ERROR: item %s unknown\n' %
                                 (PROG_NAME, opts['set_item']))
                exit(1)
        gps_model.send_cfg_valset(nvs)

    exit_code = io_handle.read(opts)

    if ((VERB_RAW <= opts['verbosity']) and io_handle.out):
        # dump raw left overs
        print("Left over data:")
        print(io_handle.out)

    sys.stdout.flush()
    io_handle.ser.close()

except KeyboardInterrupt:
    print('')
    exit_code = 1

sys.exit(exit_code)
