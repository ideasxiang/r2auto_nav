import time
import RPi.GPIO as GPIO
import serial

GPIO.setmode(GPIO.BCM)
servo_pin = 13 
GPIO.setup(servo_pin, GPIO.OUT)
p= GPIO.PWM(servo_pin, 50)
p.start(2.5)


angle=36
temperature=[]
angle=[]

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=2)

while True:
    readText = ser.readline()
    decode = readText.decode('UTF-8')
    print(decode)

def servo(tmp,tmp_am):
   try:
#-------------------adjusting the angle---------------------   
   
     while (temp - temp_am > 1):     
       for i in range (10):
         pwm = 5.0*angle/90.0+2.5
         p.ChangeDutyCycle(pwm)
         time.sleep(1)
         temp=get_am_tmp()
         an=i*36
         temperature.append(temp)
         angle.append(an)
       
       max_tmp = max(temperature)
       index=temperature.index(max_tmp)
       max_an = angle[index]
       
       pwm = 5.0*max_an/90.0+2.5
       p.ChangeDutyCycle(pwm)
       time.sleep(1)  
#---------------------------------------------------------------


#---------------------shooting--------------------------------



#------------------------------------------------------------
   except KeyboardInterrupt:
       p.stop()
       GPIO.clenup()


