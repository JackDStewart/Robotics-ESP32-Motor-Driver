import argparse
import struct
import serial
import time

SOF = 0xAA55

# encoder_data_t:
#   int16_t dL
#   int16_t dR
#   uint32_t timestamp_ms
ENCODER_FMT = "<hhI"
ENCODER_SIZE = struct.calcsize(ENCODER_FMT)  # 8 bytes

# data_packet_t (packed):
#   uint16_t SOF
#   uint8_t  seq
#   encoder_data_t encoder_data
#   uint16_t checksum
PACKET_FMT = "<HB" + "hhI" + "H"
PACKET_SIZE = struct.calcsize(PACKET_FMT)    # 13 bytes


def calc_checksum_like_c(packet_bytes: bytes) -> int:
    """
    Matches your C checksum IF you zero the checksum field before computing.
    In C you should do:
        packet->checksum = 0;
        packet->checksum = calculate_checksum(buffer, sizeof(data_packet_t));
    """
    s = 0
    for b in packet_bytes:
        s += b
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def find_sof(buf: bytearray) -> int:
    # SOF=0xAA55 as uint16 little-endian -> bytes 55 AA
    return buf.find(struct.pack("<H", SOF))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0, /dev/ttyACM0, /dev/cu.usbmodem*, COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"Listening on {ser.port} @ {ser.baudrate} baud")
    print(f"Expecting PACKET_SIZE={PACKET_SIZE} bytes (SOF+seq+dL+dR+timestamp+checksum)")
    print("Press Ctrl+C to stop.\n")

    buf = bytearray()

    try:
        while True:
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)

            while True:
                i = find_sof(buf)
                if i < 0:
                    # prevent runaway buffer if no SOF found
                    if len(buf) > 8192:
                        del buf[:-2]
                    break

                # drop bytes before SOF
                if i > 0:
                    del buf[:i]

                # need full packet
                if len(buf) < PACKET_SIZE:
                    break

                packet_bytes = bytes(buf[:PACKET_SIZE])
                del buf[:PACKET_SIZE]

                # unpack
                sof, seq, dL, dR, timestamp_ms, rx_checksum = struct.unpack(PACKET_FMT, packet_bytes)

                # verify checksum by zeroing checksum bytes then computing
                tmp = bytearray(packet_bytes)
                tmp[-2:] = b"\x00\x00"
                computed = calc_checksum_like_c(bytes(tmp))
                ok = (computed == rx_checksum)

                print(
                    f"seq={seq:3d}  "
                    f"dL={dL:6d}  dR={dR:6d}  "
                    f"t={timestamp_ms:10d} ms  "
                    f"checksum={'OK' if ok else 'BAD'} (rx=0x{rx_checksum:04X} calc=0x{computed:04X})"
                )

            if not chunk:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
