import usb.core
import usb.util 
import time 
from adafruit_servokit import ServoKit
import numpy as np 
import matplotlib.pyplot as plt
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
    KD = 0.05
    KI = 0.0
    KP = 0.6

    err = sp - pv 
    iErr = iErr + KI*err*dt
    dErr = (cv - pv)/dt

    u = KP*err + KI*iErr - KD*dErr

    return u

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
fMx = np.zeros(windowSize)
fMy = np.zeros(windowSize)
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
butterFilt = False  
butterFilt_u = True 
#Butterworth Filter Setup 
normal_cutoff = cutoff/nyq 
normal_cutoff = 0.1


i = 0
#Initial computation of dt 
start_time = time.time()
time.sleep(0.1)
current_time = time.time()
dt = current_time - start_time


n = 10000
Data = np.zeros((n,3))

start_time = time.time()
for i in range(n):
  #Read touchscreen Data
  x, y = touchScreen_data(dev, ep_in, ep_out)
  current_time = time.time()
  dt = current_time - start_time
  Data[i,0] = dt
  Data[i,1] = x
  Data[i,2] = y


pd.DataFrame(Data).to_csv('Data.csv')