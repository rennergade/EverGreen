import binascii
import sys
import os
import struct
import array


def CRC32(filename):
    buf = open(filename,'rb').read()
    print len(buf)
    ret_val = binascii.crc32(buf) & 0xFFFFFFFF
    print hex(ret_val)
    return ret_val

def filesize(filename):
    info = os.stat(filename)
    return info.st_size

def versionbin(version):
    verbytes = []
    version = version.split(".")
    byte1 = (0 << 4) | int(version[0], 2)
    verbytes.append(byte1)
    byte2 =(int(version[2], 2) << 4) | int(version[2], 2)
    verbytes.append(byte2)



    return verbytes

def main(filename, version):
    data = array.array('B')

    verbytes = versionbin(version)
    for v in verbytes:
        data.append(v)

    crc = CRC32(filename)
    data.append((crc & 0xff))
    data.append((crc >> 8) & 0xff)
    data.append((crc >> 16) & 0xff)
    data.append((crc >> 24) & 0xff)


    metafile = file('metadata.bin', 'wb')
    data.tofile(metafile)
    metafile.close()

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
