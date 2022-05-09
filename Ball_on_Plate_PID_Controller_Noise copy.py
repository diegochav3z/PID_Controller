import usb.core
import usb.util 
import time 
from adafruit_servokit import ServoKit
import numpy as np 
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import pandas as pd

#Touchscreen Initialization
def touchScreen_init(): 
  dev = usb.core.find(idVendor = 0x04d8, idProduct = 0x0c02)
  ep_in = dev[0].interfaces()[0].endpoints()[0]
  ep_out = dev[0].interfaces()[0].endpoints()[1]
  intf = dev[0].interfaces()[0].bInterfaceNumber
  dev.reset()
  #Check if the touchscreen has been found 
  if dev is None: 
    raise ValueError('Device not found')
  else:
    print(dev)
  #Claim the USB interface from the computer 
  if dev.is_kernel_driver_active(intf):
    dev.detach_kernel_driver(intf)
    usb.util.claim_interface(dev, intf)
  return dev, ep_in, ep_out
  
def touchScreen_data(dev, ep_in, ep_out):
  #Screen constants 
  screen_x_lim_up = 1870
  screen_x_lim_down = 1917
  screen_y_lim_up = 3920
  screen_y_lim_down = 1666
  try: 
    data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize) #Collected data from the touchscreen
    X_coordinate = (((data[2] - 8)*256 + data[1])*-1)/(screen_x_lim_down)
    Y_coordinate = (((data[4] - 8)*256 + data[3])*-1)/(screen_y_lim_down)
  except usb.core.USBError as e: 
    data = None

  return [X_coordinate, Y_coordinate]

def pid(sp, cv, pv, iErr, dt):
    #K Values
    KD = 0.1
    KI = 0.0
    KP = 0.8

    err = sp - pv 
    iErr = iErr + KI*err*dt
    dErr = (cv - pv)/dt

    u = KP*err + KI*iErr - KD*dErr

    return u

def moving_average(x, w): 
  return np.convolve(x, np.ones(w), 'valid')/w 
#Initial Setup Values for the PID
Kc = 5
tauI = 0.1
tauD = 0.1
iErr = 0 
dt = 0.01
#Initial Setup Values for the noise filter
T = dt
fs = 20 # sample rate Hz
cutoff = 8
nyq = 0.5 * fs 
order = 3
#Initial setup variables for the process
windowSize = 15
n = 1000*windowSize
tm = np.array([])
Mx = np.array([])
My = np.zeros([])
Sx = np.zeros(windowSize)
Sy = np.zeros(windowSize)
fMx = np.zeros(windowSize)
fMy = np.zeros(windowSize)
Error_x = np.zeros(windowSize)
Error_y = np.zeros(windowSize)
SP = np.zeros(windowSize)
Ux = np.zeros(windowSize)
Uy = np.zeros(windowSize)

#Initialization of the touchscreen adn the servos
[dev, ep_in, ep_out] = touchScreen_init()
kit = ServoKit(channels = 16)
kit.servo[0].angle = 90
kit.servo[2].angle = 90 

#Flags for filters and Servos
control = True 
convFilter_u = False
convFilter_m = False  
butterFilt = True   
butterFilt_u = False
movAverage = True  

#Butterworth Filter Setup 
normal_cutoff = cutoff/nyq 
normal_cutoff = 0.1
b, a = butter(order, 0.1, btype = 'low')

i = 0
#Initial computation of dt 
start_time = time.time()
time.sleep(0.1)
current_time = time.time()
dt = current_time - start_time

n = 10000
Data = np.zeros((n,3))


while True:
  try:
    start_time = time.time()
    #Read touchscreen Data
    x, y = touchScreen_data(dev, ep_in, ep_out) # Read the touchscreen

    Mx = np.append(Mx, x)
    My = np.append(My, y)
    if i >= windowSize: 
      if convFilter_m == True:
        box_pts = windowSize
        box = np.ones(box_pts)/(box_pts)
        fmx = np.convolve(Mx[-windowSize:], box, mode = 'same')
        fmy = np.convolve(My[-windowSize:], box, mode = 'same')
      elif butterFilt == True: 
        fmx = filtfilt(b, a, Mx[-windowSize:])
        fmy = filtfilt(b, a, My[-windowSize:])
        mmmfx = moving_average(fmx, 9)[-3]
        mmmfy = moving_average(fmy, 9)[-3]
      else:
        fmx = Mx
        fmy = My 
      #Add Filtered data to the array 
      fMx = np.append(fMx, mmmfx)
      fMy = np.append(fMy, mmmfy) 
        
      #PID Computation
      ux = pid(SP[i], fMx[i], fMx[max(0,i-1)], iErr, dt)
      uy = pid(SP[i], fMy[i], fMy[max(0,i-1)], iErr, dt)
      #Wrap up of values within the limits
      if ux > 1.1 or ux < -1.1:
        ux = 1 
      if uy > 1.1 or uy < -1.1: 
        uy = 1
      Ux = np.append(Ux, ux)
      Uy = np.append(Uy, uy)

      if movAverage == True: 
        mmx = moving_average(Ux[-windowSize:], 9)[-3]
        mmy = moving_average(Uy[-windowSize:], 9)[-3]
        Sx = np.append(Sx, mmx)
        Sy = np.append(Sy, mmy)

      if convFilter_u == True:
        box_pts = windowSize
        box = np.ones(box_pts)/(box_pts)
        Ux[i] = np.convolve(Ux[-windowSize:], box, mode = 'same')[-1:]
        Uy[i] = np.convolve(Uy[-windowSize:], box, mode = 'same')[-1:]
      if butterFilt_u == True: 
        Ux[i] = filtfilt(b, a, Ux[-windowSize:])[-5]
        Uy[i] = filtfilt(b, a, Uy[-windowSize:])[-5]
         

      #dt calculation
      current_time = time.time()
      dt = current_time - start_time
      if control == True and i%30 == 0: 
        #Servo signal X 
        if Sx[i] >= 0: 
          kit.servo[2].angle = 1*Sx[i]*90+90
        elif Sx[i] < 0:
          kit.servo[2].angle = 1*Sx[i]*90+90
        #Servo Signal Y
        if Sy[i] < 0:
          kit.servo[0].angle = 1*Sy[i]*90+90
        elif Sy[i] >= 0:
          kit.servo[0].angle = 1*Sy[i]*90+90
        
    SP = np.append(SP, 0)
    tm = np.append(tm, i)
    print(i)

    #print('Time','Servo', 'Ball Position', 'Setpoint')
    #print(i, f'{Uy[i]:2.2f},{My[i]:2.2f},{SP[i]:2.2f}')
    i = i + 1
    Data[i,0] = dt
    Data[i,1] = x
    Data[i,2] = y
  except KeyboardInterrupt:
    #Plots
    plt.figure(figsize= (15,10))
    plt.rcParams.update({'font.size': 14})
    plt.plot(tm, fMx[:tm.shape[0]], linewidth = 1.0)
    plt.plot(tm, Mx[:tm.shape[0]], linestyle = 'dashed',linewidth = 1.0)
    plt.plot(tm, Ux[:tm.shape[0]], linewidth = 1.0)
    plt.plot(np.arange(len(Sx)), Sx)
    ax = plt.gca()
    plt.xlabel('Timestep', fontsize = 24)
    plt.ylabel('Value', fontsize = 24)
    plt.grid(True)
    plt.legend(['filtered', 'Raw', 'Control', 'Filtered Control'])

    plt.figure(figsize= (15,10))
    plt.rcParams.update({'font.size': 14})
    plt.plot(tm, Ux[:tm.shape[0]], linewidth = 1.0)
    ax = plt.gca()
    plt.xlabel('Timestep', fontsize = 24)
    plt.ylabel('Value', fontsize = 24)
    plt.grid(True)
    plt.legend(['Control Signal'])

    plt.show()
    pd.DataFrame(Data).to_csv('Data.csv')


    






