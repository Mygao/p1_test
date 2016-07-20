import struct
import mcu_protocol


def doParse(serial_handle):
    #little endian '<'
    #big endian '>'
    data = serial_handle.read(1)
    if len(data) == 0:
        return "em", ""

    header = ord(data[0])
    #print "@%x" % header
    if header != mcu_protocol.HEADER:
        print 'header is unmatching'
        return "em", ""

    data = serial_handle.read(1)
    if len(data) == 0:
        print 'data length is 0'
        return "em", ""

    #length = struct.unpack('<B', data[0])
    length = ord(data[0])

    data_stripped = serial_handle.read(length + 1)
    #print len(data_stripped)
    command, = struct.unpack('<B', data_stripped[0])
    #print command
    #for c in data_stripped:
    #   print 'D %d' % (ord(c))
    return command, data_stripped
