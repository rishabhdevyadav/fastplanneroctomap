import csv
import numpy as np
import matplotlib.pyplot as plt

with open('mobilerobotpose-vrpn_client_node-RigidBody002-pose.csv') as csv_file:
    print("file read")
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    abs_time = []
    time_ = []
    pos_x_ = []
    pos_y_ = []
    for row in csv_reader:
        if line_count == 0:
            # print(f'Column names are {", ".join(row)}')
            print(row[2])
            line_count += 1
        else:
            # print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
            # print(row)
            time_.append(float(row[2]) + float(row[3])/1000000000)
            pos_x_.append(float(row[5]))
            pos_y_.append(float(row[6]))
            # abs_time.append(float(row[0]))
            line_count += 1
    print('file read')


pos_x = np.array(pos_x_)
pos_y = np.array(pos_y_)
time = np.array(time_)
time = time-time[0]

# print(np.max(pos_x))
offset_x = (np.max(pos_x) + np.min(pos_x))/2
offset_y = (np.max(pos_y) + np.min(pos_y))/2
pos_x = (pos_x - offset_x)/1.5 
pos_y = (pos_y - offset_y)/1.5 

plt.plot(time,pos_x,label='x')
plt.plot(time,pos_y,label='y')
plt.legend()
plt.show()

