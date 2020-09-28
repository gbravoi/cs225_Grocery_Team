#!/usr/bin/env python3

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read 
file_name_e1 = "./question_e1.txt"
file_name_e2 = "./question_e2.txt"
file_name_f1 = "./question_f1.txt"
file_name_f2 = "./question_f2.txt"
file_name_g = "./question_g.txt"

mass_e1 = np.loadtxt(file_name_e1 ,skiprows=0)
mass_e2 = np.loadtxt(file_name_e2 ,skiprows=0)
grav_f1 = np.loadtxt(file_name_f1 ,skiprows=0)
grav_f2 = np.loadtxt(file_name_f2 ,skiprows=0)
grav_g = np.loadtxt(file_name_g ,skiprows=0)


q3_dense = np.linspace(-90,90,251)
q3 = (-90, -60, -30, 0, 30, 60, 90)

d2_dense = np.linspace(0,2,251)
d2 = (0, 0.5, 1, 1.5, 2)

# question e
fig0 = plt.figure(0,figsize=(7,9))

ax1 = fig0.add_subplot(211)
ax1.plot(q3_dense, mass_e1[:,0], label=r'$m_{11}$')
ax1.plot(q3_dense, mass_e1[:,1], label=r'$m_{22}$')
ax1.plot(q3_dense, mass_e1[:,2], label=r'$m_{33}$')

ax1.set_xticks(q3)
ax1.set_xlabel(r'$\theta_3 (degrees)$')
ax1.set_ylabel('Mass matrix element ($m^2kg$-$kg$-$m^2kg$)')
ax1.set_title(r'Effect of varying $\theta_3$')

ax1.legend(loc=(0.8,0.2),frameon=1)

ax2 = fig0.add_subplot(212)
ax2.plot(d2_dense, mass_e2[:,0], label=r'$m_{11}$')
ax2.plot(d2_dense, mass_e2[:,1], label=r'$m_{22}$')
ax2.plot(d2_dense, mass_e2[:,2], label=r'$m_{33}$')

ax2.set_xticks(d2)
ax2.set_xlabel(r'$d_2 (meters)$')
ax2.set_ylabel('Mass matrix element ($m^2kg$-$kg$-$m^2kg$)')
ax2.set_title(r'Effect of varying $d_2$')

lgd = ax2.legend(loc=(0.8,0.2),frameon=1)

# question f
fig1 = plt.figure(1,figsize=(7,9))

ax12 = fig1.add_subplot(211)
ax12.plot(q3_dense, grav_f1[:,0], label=r'$G_{1}$')
ax12.plot(q3_dense, grav_f1[:,1], label=r'$G_{2}$')
ax12.plot(q3_dense, grav_f1[:,2], label=r'$G_{3}$')

ax12.set_xticks(q3)
ax12.set_xlabel(r'$\theta_3 (degrees)$')
ax12.set_ylabel('Gravitational torque ($Rad/s^2$-$m/s^2$-$Rad/s^2$)')
ax12.set_title(r'Effect of varying $\theta_3$')

ax12.legend(loc=(0.8,0.2),frameon=1)

ax22 = fig1.add_subplot(212)
ax22.plot(d2_dense, grav_f2[:,0], label=r'$G_{1}$')
ax22.plot(d2_dense, grav_f2[:,1], label=r'$G_{2}$')
ax22.plot(d2_dense, grav_f2[:,2], label=r'$G_{3}$')

ax22.set_xticks(d2)
ax22.set_xlabel(r'$d_2 (meters)$')
ax22.set_ylabel('Gravitational torque ($Rad/s^2$-$m/s^2$-$Rad/s^2$)')
ax22.set_title(r'Effect of varying $d_2$')

lgd = ax22.legend(loc=(0.8,0.2),frameon=1)

# question g
d4_dense = np.linspace(0,1,251)
d4 = (0, 0.5, 1)

fig2 = plt.figure(2,figsize=(7,4))

plt.plot(d4_dense, grav_g[:,0], label=r'$G_{1}$')
plt.plot(d4_dense, grav_g[:,1], label=r'$G_{2}$')
plt.plot(d4_dense, grav_g[:,2], label=r'$G_{3}$')
plt.plot(d4_dense, grav_g[:,3], label=r'$G_{4}$')

plt.xticks(d4)
plt.xlabel(r'$d_4 (meters)$')
plt.ylabel('Gravitational torque ($Rad/s^2$-$m/s^2$-$Rad/s^2$-$m/s^2$)')
plt.title(r'Effect of varying $d_4$')

lgd = plt.legend(loc=(0.8,0.7),frameon=1)

plt.show()
