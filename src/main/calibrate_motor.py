import sys
sys.path.append('../library/')
from selemod import Actuator

pin_esc = 18
pin_servo = 23
# pin_servo_2 = 24
freq_esc = 50 
freq_servo = 50
brakeon_duty = 8.72 
brakeoff_duty = 4.85

pin_ec2_top = 16 
pin_ec2_bottom = 20

actu = Actuator(pin_esc=pin_esc, pin_servo=pin_servo, 
                freq_esc=freq_esc, freq_servo=freq_servo, 
                brakeon_duty=brakeon_duty, brakeoff_duty=brakeoff_duty)


actu.calibrate_esc()

