import serial
import sys
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtWidgets

# app = QtWidgets.QApplication([])
# ser = serial.Serial("/dev/tty.usbmodem1103", 115200, timeout=1)

# tim = []
# acc_x = []
# acc_y = []
# acc_z = []
# gyr_x = []
# gyr_y = []
# gyr_z = []

# win = pg.GraphicsLayoutWidget(show=True, title="Live IMU Data")
# p_acc = win.addPlot(title="Acclerometer (g)")
# win.nextRow()
# p_gyr = win.addPlot(title="Gyroscope (deg/s)")

# curve_ax = p_acc.plot(pen='r')
# curve_ay = p_acc.plot(pen='g')
# curve_az = p_acc.plot(pen='b')
# curve_gx = p_gyr.plot(pen='c')
# curve_gy = p_gyr.plot(pen='m')
# curve_gz = p_gyr.plot(pen='y')


# def update_serial_data():
#     reads = 0
#     max_reads = 100

#     while ser.is_open and ser.in_waiting and reads < max_reads:
#         line = ser.readline().decode('utf-8', errors='ignore').strip()
#         data = line.split(",")
#         print(data)

#     #     tim.append(float(data[0]))
#     #     acc_x.append(float(data[1]))
#     #     acc_y.append(float(data[2]))
#     #     acc_z.append(float(data[3]))
#     #     gyr_x.append(float(data[4]))
#     #     gyr_y.append(float(data[5]))
#     #     gyr_z.append(float(data[6]))

#     #     if len(tim) > max_reads:
#     #         del tim[0], acc_x[0], acc_y[0], acc_z[0]
#     #         del gyr_x[0], gyr_y[0], gyr_z[0]

#     #     reads+=1

#     # if tim:
#     #     curve_ax.setData(tim, acc_x)
#     #     curve_ay.setData(tim, acc_y)
#     #     curve_az.setData(tim, acc_z)
#     #     curve_gx.setData(tim, gyr_x)
#     #     curve_gy.setData(tim, gyr_y)
#     #     curve_gz.setData(tim, gyr_z)

# timer = QtCore.QTimer()
# timer.setInterval(1)
# timer.timeout.connect(update_serial_data)
# timer.start()

# if __name__ == '__main__':
#     update_serial_data()

ser = serial.Serial("/dev/tty.usbmodem1103", 115200, timeout=1)

while ser.is_open:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    data = line.split(",")
    print(data)