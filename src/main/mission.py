import sys 
import RPi.GPIO as gpio 
import smbus
import spidev
from time import sleep
import selemod
from math import pi 

from selemod import Actuator, Bme280, Sht31, E2S, Encoder, LS7366R, TWILITE_REMOTE, EM_SW

import threading # if unable to import, use "pip3 install thread6 in terminal"


class Resilience: 
    def __init__(self, DISTANCE:int, REDUCE_RATE: float, SPEC:dict, sensor:dict): 
        """
        Args
        --------------------------------------------------------
        DISTANCE(int) : maximum altitude of climber (100m)
        REDUCE_RATE(float) : reduce rate of climber falling sequence (0.05)
        SPEC(dict) : {"radius":radius(int), "height":height(int)}
        sensor(dict) : {"bme" : bool, "sht" : bool, "counter" : bool}

        usage
        ---------------------------------------------------------------------------------
        set what's related to the mission (e.g. constants, class object, setup method etc)
        """

        # physical info about the climber 
        self.DISTANCE = DISTANCE
        self.REDUCE_RATE = REDUCE_RATE
        #self.RADIUS, self.HEIGHT = SPEC["radius"], SPEC["height"], SPEC["gear_ratio_enc2roller"]
        self.ENC_COEFFICIENT = - 2 * pi * SPEC["radius"] * SPEC["gear_ratio_enc2roller"]
        self.bme_is_use, self.sht_is_use, self.counter_is_use = sensor["bme"], sensor["sht"], sensor["counter"]

        # pin setup 
        self.pin_esc = 18 
        self.pin_servo = 12
        self.pin_esc_raspi = 13 
        self.pin_servo_raspi = 19
        # Trigger raspii
        self.trig_raspi = 5
        self.trig_led = 26

        # e2s pin setup 
        self.pin_e2s_top = 16 
        self.pin_e2s_bottom = 20 
        #self.pin_micro_sw_top = 16 
        #self.pin_micro_sw_bottom = 20 
        # emergency switch pin setup 
        self.pin_em_sw = 21

        # remote operation (TWILITE) pin setup 
        self.digital_in_1 = 17
        self.digital_in_2 = 27
        #self.digital_out_1 = 23
        #self.digital_out_2 = 24
        self.pin_remote_actuate = 23
        self.pin_remote_stop    = 24
        # rotary encoder counter IC pin setup 
        self.pin_rst = 25
        self.pin_int = 12
        
        # motor motion setup
        self.freq_esc = 50 
        self.freq_servo = 50 
        self.brakeon_duty = 5.5
        self.brakeoff_duty = 8
        self.current_throttle = 0
        self.target_throttle = 0  
        self.lower_lim = 0 
        self.middle_lim1 = 0.6 * self.DISTANCE
        self.middle_lim2 = 0.8 * self.DISTANCE
        self.upper_lim = 0.9 * self.DISTANCE
        self.throttle_A = 70
        self.throttle_B = 55
        self.throttle_C = 60
        self.throttle_slowdown = 22
        self.throttle_D = -40 # if heli-mode cannot be used, use low rpm throttle instead  

        # instantiation 
        print("pin_servo: %.1f\n" %self.pin_servo)

        self.actu = selemod.Actuator(pin_esc=self.pin_esc, pin_servo=self.pin_servo, 
                        freq_esc=self.freq_esc, freq_servo=self.freq_servo, 
                        brakeoff_duty=self.brakeoff_duty, brakeon_duty=self.brakeon_duty) 
        self.e2s = E2S(self.pin_e2s_top, self.pin_e2s_bottom) 
        self.em_sw = EM_SW(self.pin_em_sw) 
        self.twilite_remote = TWILITE_REMOTE(self.pin_remote_actuate, self.pin_remote_stop) 
        
        if self.bme_is_use: 
            self.bme280 = Bme280(0x76) #bme280 sensor
        if self.sht_is_use: 
            self.sht31 = Sht31(0x45) #sht31 sensor
        if self.counter_is_use: 
            self.ls7366r = LS7366R(0, 1000000, 4, self.pin_rst, self.pin_int) #spi ce0 & byte mode=4


        # encoder pin setup & count, pos
        # these variable are crucial to the mission, so manage these as instance variable of class git 
        # (not using as an argument so to manage across method)
        self.count = 0 
        self.pos = 0 
        # mode of esc 
        # 0 => elevation  1 => free fall 
        self.mode = 0 
        
        print("\nCondition checking...\n")
        #calibration setup 
        self.actu.brakeon() #servo brake off b4 climbing 
        print("Brake is ready.\n")
        self.actu.set_default_throttle()       
        
        print("Sensor checking...")
        em_flag     = 1
        rmstop_flag = 1
        while em_flag == 1 or rmstop_flag == 0:
            try:
                em_flag  = self._em_sw()
                rmstop_flag = self._remote_stop()
 
                if em_flag == 1:
                    print("turn off Emergency switch.")
                    sleep(3)
                if rmstop_flag == 0:
                    print("turn off remote stop switch.")
                    sleep(3)
            except KeyboardInterrupt: 
                print("Aborting the sequence")
                sys.exit()
        
        print("Sensor clear!\n")


    def motor(self, e2s_flag, em_flag, rmstop_flag): 
        """
        Arguments
        -------------------------------------------------------------------
        e2s_flag(tuple) : (top e2s flag, bottom e2s flag)
        em_flag(int) : binary flag(0/1) to notify if emergency switch is on 

        Usage
        ------------------------------------------------------------------------------------
        This method only changes throttle value based on encoder count value. 
        Also stop motor based on counter value, e2s emergency signal, emergency switch signal
        """ 

        # suppose the entire path is categorized into 3 parts: begging, middle, ending
        # the user can create new category
        # if so, add position domain like middle_lim1, middle_lim2, middle_lim3, ... 
        # throttle value of each domain is initialized in construction 

        if self.mode == 0:
            if self.lower_lim <= self.pos < self.middle_lim1:
                txt = "mode A"
                self.target_throttle = self.throttle_A
            elif self.middle_lim1 <= self.pos < self.middle_lim2:
                txt = "mode B"
                self.target_throttle = self.throttle_B
            elif self.middle_lim2 <= self.pos < self.upper_lim:
                txt = "mode C"
                self.target_throttle = self.throttle_C

            if self.current_throttle != self.target_throttle: #change throttle value only if current throttle and target throttle is different
                self.actu.new_throttle(self.target_throttle)
                self.current_throttle = self.target_throttle
                print("mode change: ", txt)
            
            #### esc stop sequence ####
            # if near the goal, stop esc (this area is above safe zone, so immediately set throttle 0 once the climber reach this area)
            if self.upper_lim <= self.pos: 
                #print("Finish! Brake is turned on.")
                #self.actu.stop_esc(self.current_throttle)
                #self.actu.check_brake()
                #gpio.cleanup()
                #sys.exit()
                print("climber near the goal")
                #self.actu.new_throttle(self.throttle_slowdown)
                #sleep(5)
                self.actu.stop_esc(self.current_throttle)
                self.actu.brakeoff()
                self.mode = 1 
                self.maxReachHeight = self.pos
                print("switching to mode 1")
                sleep(10)
                self.actu.brakeon()

            # Proximity switch (E2S) stop 
            e2s_0_flag, e2s_1_flag = e2s_flag
            if e2s_0_flag==1: 
            # Top E2S
                txt = "top e2s ON"
                self._stop_sequence_decsend(txt)
            
            if e2s_1_flag==1:
            # Bottom E2S
                txt = "bottom e2s ON"
                self._stop_sequence(txt)
            
            # Remote switch (TWILITE) stop9
            if rmstop_flag == 0: 
                txt = "remote stop switch ON"
                self._stop_sequence(txt)
            
            # Emergency switch stop
            if em_flag == 1: 
                txt = "emergency switch ON"
                self._stop_sequence(txt)
            

        # once self.mode is set to 1 (climb down), never change self.mode to 0 (for safety reason)
        # while self.mode = 1, continue heli-mode -> change to normal mode and set throttle 0 
        elif self.mode == 1: 
            #swith to heli-mode every 5% of DISTANCE
            if self.current_throttle != self.throttle_D: 
                self.actu.new_throttle(self.throttle_D)
                txt = "mode D"
                print("mode change: ", txt)
                print("setting throttle : %.1f\n" %self.throttle_D)
                self.current_throttle = self.throttle_D


            if self.pos < self.maxReachHeight*self.REDUCE_RATE: 
                txt = "turning off motor and activate brake for 5sec"
                self._stop_sequence(txt)

    def _stop_sequence(self,txt):
        print(txt)
        self.actu.stop_esc(self.current_throttle)
        print("turning off actuator")
        print("Final position status : count {},  position {}".format(self.count, self.pos))
        self.actu.brakeoff()
        #self.actu.check_brake()
        gpio.cleanup()
        sys.exit()                

    def _stop_sequence_decsend(self,txt):
        print(txt)
        self.actu.stop_esc(self.current_throttle)

        self.mode = 1 
        self.maxReachHeight = self.pos
        print("Top switch on! Switching to mode 1")
        sleep(1)
        self.actu.new_throttle(self.throttle_D)
        txt = "mode D"
        print("mode change: ", txt)
        print("setting throttle : %.1f\n" %self.throttle_D)
        self.current_throttle = self.throttle_D

        if self.pos < self.maxReachHeight*self.REDUCE_RATE: 
            txt = "turning off motor and activate brake for 5sec"
            self._stop_sequence(txt)
        
    def brake(self, servo_flag):
         """ 
         sevo_flag(int) : binary integer flag whether turning on brake or not 
         """
         if servo_flag==0: 
             self.actu.ser_1.brakeon()
         elif servo_flag==1: 
             self.actu.ser_1.brakeon() 

    def _e2s(self): 
        e2s_0_flag = self.e2s.read_top()
        e2s_1_flag = self.e2s.read_bottom()
        return (e2s_0_flag, e2s_1_flag)

    def _em_sw(self): 
        em_flag = self.em_sw.read()
        return em_flag
    
    def _remote_stop(self): 
        rmstop_flag = self.twilite_remote.read_stop()
        return rmstop_flag
    
    def _encoder(self): 
        '''based on encoder count value, compute climber's position'''
        self.rotary_rate = self.ls7366r.readRotaryRate()
        self.pos = self.ENC_COEFFICIENT * self.rotary_rate
        print("Position: {:.2f}" .format(self.pos))

    def _bme280(self): 
        press, temp, humid = self.bme280.read()
        return (press, temp, humid)

    def _sht31(self): 
        temp, humid = sht31.read()
        return (temp, humid)

    def run(self):
        # main program cc

        # process _encoder function in another thread
        #enc_thread = threading.Thread(target=self._encoder)
        #enc_thread.start()
        #enc_thread.setDaemon(True)

        while True: 
            try: 
                em_flag  = self._em_sw()
                e2s_flag = self._e2s()
                rmstop_flag = self._remote_stop()
                
                self.motor(e2s_flag, em_flag, rmstop_flag)
                self._encoder()
                sleep(0.1)      # Less than 0.1 s might cause sampling error
            except KeyboardInterrupt: 
                self.actu.stop_esc(self.current_throttle)
                txt = "Aborting the sequence"
                self._stop_sequence(txt)
