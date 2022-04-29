from asyncio import windows_utils
import usb.core
import usb.util 
import time 
from adafruit_servokit import ServoKit
import numpy as np 
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt


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
    KP = 0.4

    err = sp - pv 
    iErr = iErr + KI*err*dt
    dErr = (cv - pv)/dt

    u = KP*err + KI*iErr + KD*dErr

    return u



#Initial Setup Values
Kc = 5
tauI = 0.1
tauD = 0.1
iErr = 0 
dt = 0.01
tm = np.linspace(0, n-1, n) # timestamp
Mx = np.zeros(n)
My = np.zeros(n)
Ux = np.zeros(n) 
Uy = np.zeros(n)
SP = np.zeros(n)
fMx = np.zeros(n) 
fMy = np.zeros(n)
[dev, ep_in, ep_out] = touchScreen_init()
kit = ServoKit(channels = 16)
kit.servo[0].angle = 90
kit.servo[2].angle = 90 

#Setup variables for the noise filter 
T = dt
fs = 20 # sample rate Hz
cutoff = 2 
nyq = 0.5 * fs 
order = 2 
windowSize = 10
n = 1000*windowSize




start_time = time.time()
#filtered data 
normal_cutoff = cutoff/nyq 
b, a = butter(order, normal_cutoff, btype = 'low', analog = False)

#Obtain Data
while True:
  #Read touchscreen Data
  [Mx[i],My[i]] = touchScreen_data(dev, ep_in, ep_out) # Read the touchscreen

for i in range(n):
  if n >= windowSize:
    for ii in range(windowSize):
      



    current_time = time.time() 

    Uy[i] = pid(SP[i], My[i], My[max(0,i-1)], iErr, dt)
    Ux[i] = pid(SP[i], Mx[i], Mx[max(0,i-1)], iErr, dt)
    print(Uy[i],Ux[i])

    fUy = filtfilt(b, a, Uy[i], axis = 0)
    fUx = filtfilt(b, a, Ux[i], axis = 0)
    print(fUx)
      
      # if Ux[i] >= 0: 
      #     kit.servo[2].angle = 1*Ux[i]*90+90
      # elif Ux[i] < 0:
      #     kit.servo[2].angle = 1*Ux[i]*90+90
      
      # if Uy[i] < 0:
      #     kit.servo[0].angle = 1*Uy[i]*90+90
      # elif Uy[i] >= 0:
      #     kit.servo[0].angle = 1*Uy[i]*90+90

      #kit.servo[2].angle = U[i]

      

    elapsed_time = current_time - start_time
    start_time = current_time
    #print('Time','Servo', 'Ball Position', 'Setpoint')
    #print(elapsed_time, f'{Uy[i]:2.2f},{My[i]:2.2f},{SP[i]:2.2f}')
  time.sleep(dt)


plt.figure(figsize= (15,10))
plt.rcParams.update({'font.size': 14})
plt.plot(tm, Uy, linewidth = 1.0)
plt.plot(tm, Ux, linestyle = 'dashed',linewidth = 1.0)

ax = plt.gca()
plt.xlabel('Timestep', fontsize = 24)
plt.ylabel('Angle (degrees)', fontsize = 24)
plt.grid(True)
plt.legend(['CF roll angle', 'Accelerometer based roll angle', 'Gyroscope based roll angle'])
plt.savefig('CF_VS_ROLL.png')
plt.show()





