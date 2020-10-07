#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read 
file_name_1 = "./question1.txt"
file_name_2 = "./question2.txt"
file_name_3 = "./question3.txt"
file_name_4 = "./question4.txt"
file_name_5 = "./question5.txt"
file_name_6 = "./question6.txt"
file_name_6_bis = "./question6_bis.txt"

data_1 = np.loadtxt(file_name_1 ,skiprows=0)
data_2 = np.loadtxt(file_name_2 ,skiprows=0)
data_3 = np.loadtxt(file_name_3 ,skiprows=0)
data_4 = np.loadtxt(file_name_4 ,skiprows=0)
data_5 = np.loadtxt(file_name_5 ,skiprows=0)
data_6 = np.loadtxt(file_name_6 ,skiprows=0)
data_6_bis = np.loadtxt(file_name_6_bis ,skiprows=0)

# question 1
time_1 = np.arange(np.shape(data_1)[0])
time_1 = time_1/100
q_1 = data_1[:,0:3]
qd_1 = data_1[:,3:6]

fig0 = plt.figure(0,figsize=(7,8))

fig0.add_subplot(311)
plt.plot(time_1, q_1[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_1, qd_1[:,0], 'r--', label=r'$qd_{1}$')
plt.title('PD controller, q vs qd', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig0.add_subplot(312)
plt.plot(time_1, q_1[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_1, qd_1[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig0.add_subplot(313)
plt.plot(time_1, q_1[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_1, qd_1[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

# question 2
time_2 = np.arange(np.shape(data_2)[0])
time_2 = time_2/100
q_2 = data_2[:,0:3]
qd_2 = data_2[:,3:6]

fig1 = plt.figure(1,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_2, q_2[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_2, qd_2[:,0], 'r--', label=r'$qd_{1}$')
plt.title('PD controller with\ngravity compensation, q vs qd', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_2, q_2[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_2, qd_2[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_2, q_2[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_2, qd_2[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)


# question 3
time_3 = np.arange(np.shape(data_3)[0])
time_3 = time_3/100
q_3 = data_3[:,0:3]
qd_3 = data_3[:,3:6]

fig1 = plt.figure(2,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_3, q_3[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_3, qd_3[:,0], 'r--', label=r'$qd_{1}$')
plt.title('PD controller with gravity compensation\nand inertial compensation, q vs qd', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_3, q_3[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_3, qd_3[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_3, q_3[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_3, qd_3[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)


# question 4
time_4 = np.arange(np.shape(data_4)[0])
time_4 = time_4/100
q_4 = data_4[:,0:3]
qd_4 = data_4[:,3:6]

fig1 = plt.figure(3,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_4, q_4[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_4, qd_4[:,0], 'r--', label=r'$qd_{1}$')
plt.title('PD controller with gravity compensation\nand dynamic decoupling, q vs qd', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_4, q_4[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_4, qd_4[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_4, q_4[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_4, qd_4[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)


# question 5
time_5 = np.arange(np.shape(data_5)[0])
time_5 = time_5/100
q_5 = data_5[:,0:3]
qd_5 = data_5[:,3:6]

fig1 = plt.figure(4,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_5, q_5[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_5, qd_5[:,0], 'r--', label=r'$qd_{1}$')
plt.title('Error in payload mass', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_5, q_5[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_5, qd_5[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_5, q_5[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_5, qd_5[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.1),frameon=1)


# question 6
time_6 = np.arange(np.shape(data_6)[0])
time_6 = time_6/100
q_6 = data_6[:,0:3]
qd_6 = data_6[:,3:6]

fig1 = plt.figure(5,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_6, q_6[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_6, qd_6[:,0], 'r--', label=r'$qd_{1}$')
plt.title('Error in payload mass - compensation', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_6, q_6[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_6, qd_6[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_6, q_6[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_6, qd_6[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)


# question 6 bis
time_6_bis = np.arange(np.shape(data_6_bis)[0])
time_6_bis = time_6_bis/100
q_6_bis = data_6_bis[:,0:3]
qd_6_bis = data_6_bis[:,3:6]

fig1 = plt.figure(6,figsize=(7,8))

fig1.add_subplot(311)
plt.plot(time_6_bis, q_6_bis[:,0], 'r', label=r'$q_{1}$')
plt.plot(time_6_bis, qd_6_bis[:,0], 'r--', label=r'$qd_{1}$')
plt.title('error in payload mass - compensation\nwith velocity limit', FontSize=24)
plt.ylim((-2.0,2.0))
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(312)
plt.plot(time_6_bis, q_6_bis[:,1], 'g', label=r'$q_{3}$')
plt.plot(time_6_bis, qd_6_bis[:,1], 'g--', label=r'$qd_{3}$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
plt.ylim((-0.2,0.1))
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

fig1.add_subplot(313)
plt.plot(time_6_bis, q_6_bis[:,2], 'b', label=r'$q_{4}$')
plt.plot(time_6_bis, qd_6_bis[:,2], 'b--', label=r'$qd_{4}$')
plt.ylim((-2.26,-2.16))

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

plt.show()
