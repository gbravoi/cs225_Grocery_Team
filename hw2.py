import redis
import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack
from datetime import datetime

def hlp(x):
    return [float(i) for i in x]

num = 30000;

qs = []
xs = []
r = redis.Redis()
i = 0
while i < num:
    if(r.get("sai2::cs225a::controller_running")=="1"):
        if i == 0:
            tstart = datetime.now()
        qs.append(hlp(r.get("sai2::cs225a::panda_robot::sensors::q")[1:-1].split(',')))
        xs.append(hlp(r.get("x")[1:-1].split(',')))
        i += 1
tend = datetime.now()
qs = np.array(qs)
xs = np.array(xs)
print(xs)
ques = 2
#Q1
telapse = (tend-tstart).total_seconds()
timeaxis = np.linspace(0.0,telapse,num)
if ques == 1:
    Q, (ax1) = plt.subplots(1, 1)
    Q.suptitle(r'Controller 1 (Q1e): $kp = 200, kv_{77} = -0.19$')
    qs = qs / np.pi * 180
    ax1.plot(timeaxis,qs[:,6]);ax1.set_ylabel(r'$q^7_t(^\circ)$');ax1.set_xlabel(r'$t$');ax1.set_title(r'$f = 2.014 Hz$')
    ax1.ticklabel_format(useOffset=False)
    plt.show(Q)
#Q2
elif ques == 2:
    Q, (ax1,ax2) = plt.subplots(2, 1)
    # Q.suptitle('Controller 2 (Q2a): kp = 200, kv = 80')
    # Q.suptitle('Controller 2 (Q2c): kp = 200, kv = 80, joint KV = 15')
    # Q.suptitle('Controller 2 (Q2d): kp = 200, kv = 80, joint KV = 12')
    # Q.suptitle('Controller 3 (Q3a): kp = 200, kv = 80, joint KV = 12')
    # Q.suptitle('Controller 3 (Q4a): kp = 200, kv = 80, joint KV = 12')
    # Q.suptitle('Controller 3 (Q4b): kp = 200, kv = 80, joint KV = 12')
    # Q.suptitle('Controller 4 (Q4c): kp = 200, kv = 80, joint KV = 12')
    Q.suptitle('Controller 4 (Q4d): kp = 200, kv = 80, joint KV = 12')
    qs = qs / np.pi * 180
    ax1.set_ylabel(r'$q_t(^\circ)$');ax1.set_xlabel(r'$t$');ax1.set_title("Joint Trajectories")
    for i in range(7):
        ax1.plot(timeaxis,qs[:,i],label="q%d"%i);
    ax1.ticklabel_format(useOffset=False)
    ax2.set_ylabel(r'$X_t(m)$');ax2.set_xlabel(r'$t$');ax2.set_title("Point Trajectory")
    xyz = ['x','y','z']
    for i in range(3):
        ax2.plot(timeaxis,xs[:,i],label="%s"%xyz[i]);
    ax2.ticklabel_format(useOffset=False)
    ax1.legend()
    ax2.legend()
    plt.show(Q)
