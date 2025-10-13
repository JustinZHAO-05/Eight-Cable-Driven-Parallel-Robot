import serial

# ports = list(serial.tools.list_ports.comports())
# for p in ports:
#     # p.device 是 COM 号（如 'COM3'），p.description 是设备描述
#     print(f"{p.device} — {p.description}")

# ser0 = serial.Serial('COM8', 115200)
ser0 = serial.Serial('COM8', 115200)
ser1 = serial.Serial('COM12', 115200)
ser2 = serial.Serial('COM11', 115200)
ser3 = serial.Serial('COM13', 115200)
ser4 = serial.Serial('COM15', 115200)
ser5 = serial.Serial('COM10', 115200)
ser6 = serial.Serial('COM14', 115200)
ser7 = serial.Serial('COM9', 115200)

SER = [ser0,ser1,ser2,ser3,ser4,ser5,ser6,ser7]

for idx, ser in enumerate(SER):
    print("idx",idx)
    
