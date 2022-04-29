import usb.core
import usb.util
import time 


dev = usb.core.find(idVendor = 0x04d8, idProduct = 0x0c02)
ep_in = dev[0].interfaces()[0].endpoints()[0]
ep_out = dev[0].interfaces()[0].endpoints()[1]
intf = dev[0].interfaces()[0].bInterfaceNumber
dev.reset()

#Screen constants 
screen_x_lim_up = 3960
screen_x_lim_down = 180
screen_y_lim_up = 3920
screen_y_lim_down = 340

#Check if the device has been found
if dev is None:
    raise ValueError('Device not found')
else: 
    print(dev)
    #dev.set_configuration()

if dev.is_kernel_driver_active(intf):
    dev.detach_kernel_driver(intf)
    usb.util.claim_interface(dev, intf)
collected = 0 
attempts = 200

t_end = time.time() + 30
while time.time() < t_end: 
    try: 
        data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize) #Collected data from the touchscreen
        #Format the data
        X_coordinate = (data[1]+data[2]*256 - screen_x_lim_down)/(screen_x_lim_up - screen_y_lim_down)
        Y_coordinate = (data[3]+data[4]*256 - screen_y_lim_down)/(screen_y_lim_up - screen_y_lim_down)

        print(X_coordinate)
    except usb.core.USBError as e: 
        data = None
        if e.args == ('Operation Timed Out'):
            continue 






while collected < attempts: 
    try: 
        data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize) #Collected data from the touchscreen
        collected += 1 
        print(data)
    except usb.core.USBError as e: 
        data = None
        if e.args == ('Operation Timed Out'):




            continue
usb.util.release_interface(dev, intf)
dev.attach_kernel_driver(intf)

# epInAdd = ep_in.bEndpointAddress
# epInMax = ep_in.wMaxPacketSize
# epOutAdd = ep_out.bEndpointAddress



#assert ep_in is not None
#assert ep_out is not None

#data_out = [55]
#dev.write(epOutAdd, data_out)

# data_in = dev.read(epOutAdd, epInMax)
# RxData = ''.join([chr(x) for x in data_in])
# print(RxData)
