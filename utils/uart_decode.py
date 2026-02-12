import argparse
import struct
import serial
import time

# encoder_data_t:
#   int16_t dL   (pcnt, -32768..32767)
#   int16_t dR   (pcnt, -32768..32767)
#   uint32_t timestamp_ms
ENCODER_FMT = "<hhI"
ENCODER_SIZE = struct.calcsize(ENCODER_FMT)  # 8 bytes

# data payload (no SOF, framed with COBS + 0x00 delimiter):
#   uint8_t  seq
#   int16_t  dL
#   int16_t  dR
#   uint32_t timestamp_ms
#   uint16_t checksum
PAYLOAD_FMT = "<BhhIH"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)  # 11 bytes


def calc_checksum_like_c(packet_bytes: bytes) -> int:
    """
    Same checksum style as before:
      - sum all bytes (uint32 accumulator)
      - fold to 16 bits
      - one's complement
    Caller should zero checksum field bytes before computing.
    """
    s = 0
    for b in packet_bytes:
        s += b
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def cobs_decode(encoded: bytes) -> bytes:
    """
    Standard COBS decode (0x00 is the frame delimiter and is NOT included here).
    Raises ValueError on malformed input.
    """
    if not encoded:
        return b""

    out = bytearray()
    i = 0
    n = len(encoded)

    while i < n:
        code = encoded[i]
        if code == 0:
            raise ValueError("COBS decode error: zero code byte")
        i += 1

        copy_len = code - 1
        if i + copy_len > n:
            raise ValueError("COBS decode error: code overrun")

        if copy_len:
            out.extend(encoded[i : i + copy_len])
            i += copy_len

        # Insert a 0x00 between blocks if code != 0xFF and we're not at end
        if code != 0xFF and i < n:
            out.append(0)

    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0, /dev/ttyACM0, /dev/cu.usbmodem*, COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.2)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"Listening on {ser.port} @ {ser.baudrate} baud")
    print(f"Expecting COBS-framed payloads of PAYLOAD_SIZE={PAYLOAD_SIZE} bytes after decode")
    print("Frame delimiter: 0x00")
    print("Press Ctrl+C to stop.\n")

    buf = bytearray()

    try:
        while True:
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)

            # Process all complete frames currently in buf (frames end with 0x00)
            while True:
                try:
                    end = buf.index(0)  # first 0x00 delimiter
                except ValueError:
                    # no complete frame yet
                    if len(buf) > 65536:
                        # prevent runaway if delimiter never arrives
                        del buf[:-1024]
                    break

                frame = bytes(buf[:end])   # encoded (no delimiter)
                del buf[: end + 1]         # drop encoded frame + delimiter

                if not frame:
                    # ignore empty frames (possible if multiple delimiters)
                    continue

                try:
                    decoded = cobs_decode(frame)
                except ValueError as e:
                    print(f"COBS decode error: {e}")
                    continue

                if len(decoded) != PAYLOAD_SIZE:
                    print(f"Bad payload size after decode: {len(decoded)} (expected {PAYLOAD_SIZE})")
                    continue

                seq, dL, dR, timestamp_ms, rx_checksum = struct.unpack(PAYLOAD_FMT, decoded)

                # verify checksum by zeroing checksum bytes then computing
                tmp = bytearray(decoded)
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