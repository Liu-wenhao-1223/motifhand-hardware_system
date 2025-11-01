import serial
import struct
import time

NUM_BOARDS = 3
FSR_COUNT = 36
PAYLOAD_SIZE = NUM_BOARDS * (1 + 12 + 12 + 16 + FSR_COUNT * 2) # 每个float是4字节

def read_exact(ser, n):
    """确保完整读取 n 字节，否则返回 None"""
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def parse_payload(data):
    offset = 0
    for i in range(NUM_BOARDS):
        # id_str = struct.unpack_from('8s', data, offset)[0].decode('ascii').strip('\x00')
        id_bytes = struct.unpack_from('8s', data, offset)[0]
        # id_str = id_bytes.decode('utf-8', errors='ignore').strip('\x00')
        # offset += 8
        id_str = struct.unpack_from('B', data, offset)[0]  # 解出 uint8_t
        offset += 1

        acc = struct.unpack_from('fff', data, offset)
        offset += 12

        gyro = struct.unpack_from('fff', data, offset)
        offset += 12

        mag = struct.unpack_from('fff', data, offset)
        offset += 12

        temp = struct.unpack_from('f', data, offset)[0]
        offset += 4

        # fsr = struct.unpack_from(f'{FSR_COUNT}f', data, offset)
        # offset += FSR_COUNT * 4

        fsr = struct.unpack_from(f'{FSR_COUNT}H', data, offset)
        offset += FSR_COUNT * 2  # 每个 uint8_t 是 1 字节

        print(f"--- Module {i} ---")
        print(f"ID: {id_str}")
        print(f"Acc: ({acc[0]:.5f}, {acc[1]:.5f}, {acc[2]:.5f})")
        print(f"Gyro: ({gyro[0]:.5f}, {gyro[1]:.5f}, {gyro[2]:.5f})")
        print(f"Mag: ({mag[0]:.5f}, {mag[1]:.5f}, {mag[2]:.5f})")
        print(f"Temp: {temp:.5f}")
        print(f"FSR[0]: {fsr[0:6]}")
        print(f"FSR[1]: {fsr[6:12]}")
        print(f"FSR[2]: {fsr[12:18]}")
        print(f"FSR[3]: {fsr[18:24]}")
        print(f"FSR[4]: {fsr[24:30]}")
        print(f"FSR[5]: {fsr[30:36]}")
    print("="*40)

def read_rs485_data(port='COM9', baudrate=115200):   # 460800   COM9: big board  COM10: small board
    ser = None
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
        print(f"Listening on {port}...")

        while True:
            head1 = ser.read(1)
            if head1 != b'\xA1':
                continue
            head2 = ser.read(1)
            # print(head2)

            length_bytes = read_exact(ser, 2)
            if length_bytes is None:
                print("[!] Timeout reading length bytes")
                continue

            payload_len = struct.unpack('<H', length_bytes)[0]

            if payload_len != PAYLOAD_SIZE:
                print(f"[!] Unexpected payload size: {payload_len}, expected: {PAYLOAD_SIZE}")
                # 清掉剩余帧：payload + 尾
                read_exact(ser, payload_len + 1)
                continue

            payload = read_exact(ser, payload_len)
            if payload is None:
                print("[!] Incomplete payload, skipping.")
                continue

            frame_tail = read_exact(ser, 1)
            if frame_tail != b'\x55':
                # print(f"[!] Invalid frame tail: got {frame_tail.hex() if frame_tail else 'None'}, expected 55")
                # print("Frame end preview", payload[-10:] + frame_tail)
                
                # print(f"[!] Invalid frame tail, skipping.")
                continue

            parse_payload(payload)

    except KeyboardInterrupt:
        print("Stopped by user.")
    except Exception as e:
        print(f"[!] Error: {e}")
    finally:
        if ser:
            ser.close()

if __name__ == '__main__':
    read_rs485_data()
