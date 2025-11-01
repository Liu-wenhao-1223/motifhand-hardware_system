import serial
import csv

# === 修改以下参数 ===
PORT = 'COM5'             # 串口名称，Linux 下可能是 '/dev/ttyUSB0'   COM5 or COM10
BAUDRATE = 460800         # 波特率
NUM_LINES = 10000          # 需要读取的行数
CSV_FILE = 'E:/Engineering/SLURM/Tactile/Film-Tactile_System/Code/Supplyments/magnetometer_data_4_3.csv'  # 输出文件名

# 打开串口
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

# 打开 CSV 文件写入
with open(CSV_FILE, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    count = 0
    print("Start reading from serial...")

    while count < NUM_LINES:
        line = ser.readline().decode('utf-8').strip()

        if not line:
            continue  # 超时或空行

        try:
            # 将字符串按逗号分割成3个 float
            values = [float(x) for x in line.split(',')]
            if len(values) == 3:
                writer.writerow(values)
                count += 1
                print(f"[{count}] {values}")
        except ValueError:
            print("Invalid line:", line)  # 如果不是数字，跳过

print("Done. Data saved to", CSV_FILE)
ser.close()

















# import serial

# # 打开串口
# ser = serial.Serial(
#     port='COM5',        # 替换为你实际的串口号，如 '/dev/ttyUSB0' (Linux/Mac)
#     baudrate=460800,    # 串口波特率
#     timeout=1           # 读超时时间（秒）
# )

# print("开始接收串口数据...")

# try:
#     while True:
#         line = ser.readline().decode('utf-8').strip()  # 读取一整行，并去除换行
#         if not line:
#             continue

#         print(f"收到原始行: {line}")

#         # 尝试按逗号分割为三个值
#         try:
#             parts = [float(x) for x in line.split(', ')]
#             if len(parts) == 3:
#                 x, y, z = parts
#                 print(f"解析数据: x={x}, y={y}, z={z}")
#             else:
#                 print("⚠️ 行中数据不是三个数")
#         except ValueError:
#             print("⚠️ 无法解析为浮点数，原始内容:", line)

# except KeyboardInterrupt:
#     print("手动退出。")
# finally:
#     ser.close()
