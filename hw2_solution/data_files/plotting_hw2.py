#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read 
file_name_1 = "./question1.txt"
file_name_2a = "./question2a.txt"
file_name_2c = "./question2c.txt"
file_name_2d = "./question2d.txt"
file_name_3 = "./question3.txt"
file_name_4i = "./question4i.txt"
file_name_4ii = "./question4ii.txt"
file_name_4iii = "./question4iii.txt"
file_name_4iv = "./question4iv.txt"

data_1 = np.loadtxt(file_name_1 ,skiprows=0)
data_2a = np.loadtxt(file_name_2a ,skiprows=0)
data_2c = np.loadtxt(file_name_2c ,skiprows=0)
data_2d = np.loadtxt(file_name_2d ,skiprows=0)
data_3 = np.loadtxt(file_name_3 ,skiprows=0)
data_4i = np.loadtxt(file_name_4i ,skiprows=0)
data_4ii = np.loadtxt(file_name_4ii ,skiprows=0)
data_4iii = np.loadtxt(file_name_4iii ,skiprows=0)
data_4iv = np.loadtxt(file_name_4iv ,skiprows=0)

# question 1
time_1 = np.arange(np.shape(data_1)[0])
time_1 = time_1/100
q_1 = data_1[:,0]
qd_1 = data_1[:,1]

fig0 = plt.figure(0,figsize=(7,8))

plt.plot(time_1, q_1, 'r', label=r'$q_{1}$')
plt.plot(time_1, qd_1, 'r--', label=r'$qd_{1}$')
plt.title('q vs qd', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)
fig0.savefig("./question1.png")
plt.clf()

# question 2a
time_2 = np.arange(np.shape(data_2a)[0])
time_2 = time_2/100
x_2 = data_2a[:,0:3]
xd_2 = data_2a[:,3:6]
q_2 = data_2a[:,6:]

fig1 = plt.figure(1,figsize=(7,8))

fig1.add_subplot(211)
plt.plot(time_2, x_2[:,0], 'r-', label=r'$x$')
plt.plot(time_2, x_2[:,1], 'g-', label=r'$y$')
plt.plot(time_2, x_2[:,2], 'b-', label=r'$z$')
plt.plot(time_2, xd_2[:,0], 'r--', label=r'$x_d$')
plt.plot(time_2, xd_2[:,1], 'g--', label=r'$y_d$')
plt.plot(time_2, xd_2[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig1.add_subplot(212)
plt.plot(time_2, q_2, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig1.savefig("./question2a.png")
plt.clf()

# question 2c
time_2 = np.arange(np.shape(data_2c)[0])
time_2 = time_2/100
x_2 = data_2c[:,0:3]
xd_2 = data_2c[:,3:6]
q_2 = data_2c[:,6:]

fig2 = plt.figure(1,figsize=(7,8))

fig2.add_subplot(211)
plt.plot(time_2, x_2[:,0], 'r-', label=r'$x$')
plt.plot(time_2, x_2[:,1], 'g-', label=r'$y$')
plt.plot(time_2, x_2[:,2], 'b-', label=r'$z$')
plt.plot(time_2, xd_2[:,0], 'r--', label=r'$x_d$')
plt.plot(time_2, xd_2[:,1], 'g--', label=r'$y_d$')
plt.plot(time_2, xd_2[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig2.add_subplot(212)
plt.plot(time_2, q_2, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig2.savefig("./question2c.png")
plt.clf()

# question 2d
time_2 = np.arange(np.shape(data_2d)[0])
time_2 = time_2/100
x_2 = data_2d[:,0:3]
xd_2 = data_2d[:,3:6]
q_2 = data_2d[:,6:]

fig3 = plt.figure(1,figsize=(7,8))

fig3.add_subplot(211)
plt.plot(time_2, x_2[:,0], 'r-', label=r'$x$')
plt.plot(time_2, x_2[:,1], 'g-', label=r'$y$')
plt.plot(time_2, x_2[:,2], 'b-', label=r'$z$')
plt.plot(time_2, xd_2[:,0], 'r--', label=r'$x_d$')
plt.plot(time_2, xd_2[:,1], 'g--', label=r'$y_d$')
plt.plot(time_2, xd_2[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig3.add_subplot(212)
plt.plot(time_2, q_2, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig3.savefig("./question2d.png")
plt.clf()

# question 3
time_3 = np.arange(np.shape(data_3)[0])
time_3 = time_3/100
x_3 = data_3[:,0:3]
xd_3 = data_3[:,3:6]
q_3 = data_3[:,6:]

fig4 = plt.figure(1,figsize=(7,8))

fig4.add_subplot(211)
plt.plot(time_3, x_3[:,0], 'r-', label=r'$x$')
plt.plot(time_3, x_3[:,1], 'g-', label=r'$y$')
plt.plot(time_3, x_3[:,2], 'b-', label=r'$z$')
plt.plot(time_3, xd_3[:,0], 'r--', label=r'$x_d$')
plt.plot(time_3, xd_3[:,1], 'g--', label=r'$y_d$')
plt.plot(time_3, xd_3[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig4.add_subplot(212)
plt.plot(time_3, q_3, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_2$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig4.savefig("./question3.png")
plt.clf()

# question 4i
time_4 = np.arange(np.shape(data_4i)[0])
time_4 = time_4/100
x_4 = data_4i[:,0:3]
xd_4 = data_4i[:,3:6]
q_4 = data_4i[:,6:]

fig5 = plt.figure(1,figsize=(7,8))

fig5.add_subplot(211)
plt.plot(time_4, x_4[:,0], 'r-', label=r'$x$')
plt.plot(time_4, x_4[:,1], 'g-', label=r'$y$')
plt.plot(time_4, x_4[:,2], 'b-', label=r'$z$')
plt.plot(time_4, xd_4[:,0], 'r--', label=r'$x_d$')
plt.plot(time_4, xd_4[:,1], 'g--', label=r'$y_d$')
plt.plot(time_4, xd_4[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig5.add_subplot(212)
plt.plot(time_4, q_4, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_4$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig5.savefig("./question4i.png")
plt.clf()

# question 4ii
time_4 = np.arange(np.shape(data_4ii)[0])
time_4 = time_4/100
x_4 = data_4ii[:,0:3]
xd_4 = data_4ii[:,3:6]
q_4 = data_4ii[:,6:]

fig6 = plt.figure(1,figsize=(7,8))

fig6.add_subplot(211)
plt.plot(time_4, x_4[:,0], 'r-', label=r'$x$')
plt.plot(time_4, x_4[:,1], 'g-', label=r'$y$')
plt.plot(time_4, x_4[:,2], 'b-', label=r'$z$')
plt.plot(time_4, xd_4[:,0], 'r--', label=r'$x_d$')
plt.plot(time_4, xd_4[:,1], 'g--', label=r'$y_d$')
plt.plot(time_4, xd_4[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig6.add_subplot(212)
plt.plot(time_4, q_4, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_4$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig6.savefig("./question4ii.png")
plt.clf()

# question 4iii
time_4 = np.arange(np.shape(data_4iii)[0])
time_4 = time_4/100
x_4 = data_4iii[:,0:3]
xd_4 = data_4iii[:,3:6]
q_4 = data_4iii[:,6:]

fig7 = plt.figure(1,figsize=(7,8))

fig7.add_subplot(211)
plt.plot(time_4, x_4[:,0], 'r-', label=r'$x$')
plt.plot(time_4, x_4[:,1], 'g-', label=r'$y$')
plt.plot(time_4, x_4[:,2], 'b-', label=r'$z$')
plt.plot(time_4, xd_4[:,0], 'r--', label=r'$x_d$')
plt.plot(time_4, xd_4[:,1], 'g--', label=r'$y_d$')
plt.plot(time_4, xd_4[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig7.add_subplot(212)
plt.plot(time_4, q_4, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_4$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig7.savefig("./question4iii.png")
plt.clf()

# question 4iv
time_4 = np.arange(np.shape(data_4iv)[0])
time_4 = time_4/100
x_4 = data_4iv[:,0:3]
xd_4 = data_4iv[:,3:6]
q_4 = data_4iv[:,6:]

fig8 = plt.figure(1,figsize=(7,8))

fig8.add_subplot(211)
plt.plot(time_4, x_4[:,0], 'r-', label=r'$x$')
plt.plot(time_4, x_4[:,1], 'g-', label=r'$y$')
plt.plot(time_4, x_4[:,2], 'b-', label=r'$z$')
plt.plot(time_4, xd_4[:,0], 'r--', label=r'$x_d$')
plt.plot(time_4, xd_4[:,1], 'g--', label=r'$y_d$')
plt.plot(time_4, xd_4[:,2], 'b--', label=r'$z_d$')
plt.ylabel('Op. space position (m)', FontSize=18)
plt.title('Op. and joint space trajectories', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

fig8.add_subplot(212)
plt.plot(time_4, q_4, label=r'$q$')
plt.ylabel('Joint angles (Rad)', FontSize=18)
lgd = plt.legend(loc=(0.8,0.2),frameon=1)

plt.xlabel('Time (seconds)', FontSize=18)
lgd = plt.legend(['$q_1$', '$q_4$', '$q_3$', '$q_4$', '$q_5$', '$q_6$', '$q_7$'], loc=(0.8,0.4),frameon=1)
fig8.savefig("./question4iv.png")
plt.clf()
