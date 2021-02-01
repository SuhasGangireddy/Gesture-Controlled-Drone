from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
from dronekit import connect, VehicleMode ,LocationGlobalRelative
import time
import getch
from RPIO import PWM
th_min = 1100
th_max = 2000
r_min = 1100
r_max = 1900
p_min = 1100
p_max = 1900
y_min = 1100
y_max = 1900
a_min = 980
a_max = 2300
th =1100
r = 1520
p = 1520
y = 1520
a = False
flag = 0
BOARD.setup()
print ("connecting to vehicle....")
vehicle = connect('/dev/ttyACM0', baud = 57600)
print ("connected")

#changing vehicle mode to stabilize
print ("\nSet Vehicle.mode = (currently: %s)") % vehicle.mode.name
while not vehicle.mode=='STABILIZE':
    vehicle.mode = VehicleMode('STABILIZE')
    vehicle.flush()

print ("vehicle mode: %s") % vehicle.mode

# ARMING the vehicle
vehicle.armed = True
while not vehicle.armed:
    vehicle.armed = True
    vehicle.flush()
    print (" trying to change mode and arming ...")
    time.sleep(1)

print ("its armed")

# initialize servo objects with PWM function
roll = PWM.Servo()
pitch = PWM.Servo()
throttle = PWM.Servo()
yaw = PWM.Servo()
mode = PWM.Servo()

# start PWM on servo specific GPIO no, this is not the pin no but it is the GPIO no 
roll.set_servo(5,1520)# pin 11
pitch.set_servo(6,1520)# pin 12
throttle.set_servo(13,1100)# pin 13, pin 14 is Ground
yaw.set_servo(19,1520)# pin 15
mode.set_servo(26,1200)# pin 37
# assign global min and max values



class LoRaRcvCont(LoRa):


    print("1")
    def __init__(self, verbose=False):
        print("2")
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def start(self):
        print("3")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            sys.stdout.flush()
            

    def on_rx_done(self):
        print("4")
        global th,r,p,y
        print("\nReceived: ")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        if(payload==[]):
            payload=[1,2,3,4,5]
        print(bytes(payload).decode("utf-8",'ignore'))
        l=payload[4]
        print(l)
        
        #print "\nSet Vehicle.mode =  (currently: %s)" % vehicle.mode.name



        print ("Taking off!")
        print ("controll drone from keyboard")
        key=l
        print(type(key))
        try:
            if True:
                
                if key == 49:
                    
                    th = th + 10
                    if (th < th_min):
                        th = 1100
                        throttle.set_servo(13,th)
                    elif (th > th_max):
                        th = 2000
                        throttle.set_servo(13,th)
                    else:
                        throttle.set_servo(13,th)
                    print ('th :' + str(th))
                    
                elif key == 51:
                    th = th - 10
                    if (th < th_min):
                        th = 1100
                        throttle.set_servo(13,th)
                    elif (th > th_max):
                        th = 2000
                        throttle.set_servo(13,th)
                    else:
                        throttle.set_servo(13,th)
                    print ('th :' + str(th))

                    #yaw left
                elif key == 55:
                    yaw.set_servo(19,1350)
                    print ("yaw left")
                    time.sleep(0.3)
                    yaw.set_servo(19,1520)

                #yaw right
                elif key == 57:
                    yaw.set_servo(19,1650)
                    print ("yaw right")
                    time.sleep(0.3)
                    yaw.set_servo(19,1520)

                #roll left
                elif key == 52:
                    roll.set_servo(5,1350)
                    print ("roll left")
                    time.sleep(0.3)
                    roll.set_servo(5,1520)

                #roll right
                elif key == 54:
                    roll.set_servo(5,1650)
                    print ("roll right")
                    time.sleep(0.3)
                    roll.set_servo(5,1520)
                    
                #pitch forward
                elif key == 56:
                    pitch.set_servo(6,1650)
                    print ("pitch forward")
                    time.sleep(0.3)
                    pitch.set_servo(6,1520)
                #pitch back
                elif key == 50:
                        pitch.set_servo(6,1350)
                        print ("pitch back")
                        time.sleep(0.3)
                        pitch.set_servo(6,1520)

##                #atlitude hold
##                elif key == 'h':
##                        mode.set_servo(26,1550)
##                        time.sleep(0.5)
##                        print " mode is %s" % vehicle.mode.name
##                        
##              
##                #land mode
##                elif key == 'l':
##                        mode.set_servo(26,1800)
##                        time.sleep(0.5)
##                        print " mode is %s " % vehicle.mode.name
##                #stabilize mode
##                elif key =='5' and th<1200:
##                        mode.set_servo(26,1200)
##                        time.sleep(0.5)
##                        print "mode is %s" % vehicle.mode.name
                        
        except KeyboardInterrupt:
                throttle.set_servo(13,1100)
                while vehicle.armed:
                        vehicle.armed = False
                        print ("disarming")
                        time.sleep(1)
                        vehicle.flush()
        print ("DISARMED")
        vehicle.close()
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        k=bytes(payload).decode("utf-8",'ignore')
        return k
        
print("5")
lora = LoRaRcvCont(verbose=False)
lora.set_mode(MODE.STDBY)
l=lora.on_rx_done()
#print(l)
print("6")

#  Medium Range  Defaults after init are 434.0MHz, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on 13 dBm

lora.set_pa_config(pa_select=1)
sleep(2)
print("7")
try:
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    
    print("8")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("9")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()
print("10")
