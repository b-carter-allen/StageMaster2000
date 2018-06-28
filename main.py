import math
import datetime, time
import objects as ob
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import RPi.GPIO as GPIO
import os
import pigpio

## Variables ##
red_intensity = 255 # Frequency of red light from 0 (off) to 255 (max)
green_intensity= 255 # Frequency of green light
blue_intensity= 200 # Frequency of blue light
Root_File_Name = "StageMaster_Pictures/"

## System Setup ##
endstop_pins=[5,6,13,19,26,20]
camera_pins=[4,17,18]
rgb_pins=[23,24,25]
lighting = pigpio.pi()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for i in range(3):
    GPIO.setup(camera_pins[i],GPIO.OUT)

for i in range(6):
    GPIO.setup(endstop_pins[i], GPIO.IN, pull_up_down=GPIO.PUD_UP)

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr = 0x60)
LinearMotor = mh.getStepper(200, 1)
LinearMotor.setSpeed(30)
RotaryMotor = mh.getStepper(200,2)
RotaryMotor.setSpeed(2)


## Temp Test Functions ##

#Turn on lights, here is where the lighting ratios can be modified
def turnOnLights():
    lighting.set_PWM_dutycycle(rgb_pins[0],red_intensity) 
    lighting.set_PWM_dutycycle(rgb_pins[1],green_intensity)
    lighting.set_PWM_dutycycle(rgb_pins[2],blue_intensity)

#Turn of all lights and stop pigpio
def turnOffLights():
    for i in range(3):
        lighting.set_PWM_dutycycle(rgb_pins[i], 0)

#Makes a .txt file to reset number of trials for vial number reference and returns .txtfile name
def initial_trial_count(run_path):
    Trial_Count= open(run_path+"/Trial_Count.txt", "w")
    Trial_Count.write("1")
    Trial_Count.close()
    trial_path=run_path+"/Trial_Count.txt"
    return(trial_path)

#Reads the trial .txt file, converts into an integer, and then increments the .txt file
def read_trial_count(trial_path):
    Trial_Count = open(trial_path, "r")
    current_trial = int(Trial_Count.read())
    Trial_Count.close()
    return(current_trial)

def inc_trial_count(trial_path):
    Trial_Count = open(trial_path, "r")
    current_trial = int(Trial_Count.read())
    next_trial = str(current_trial+1)
    Trial_Count.close()
    Trial_Count = open(trial_path, "w")
    Trial_Count.write(next_trial)
    Trial_Count.close()
    
    

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
#def detect(endstop)
def detect(endstop): # Returns boolean value for whether a specific GPIO pin is obstructed
    if GPIO.input(endstop) == True:
        return(1)
    else:
        return(0)
#Function that moves the endstop right to endstop_number (1-6)
def move_linear_platform(endstop_number): 
    endstop=endstop_pins[endstop_number - 1]
    if detect(endstop_pins[5])== 1:
        turnOffMotors()
    while detect(endstop)== 0:
        LinearMotor.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)
    while detect(endstop)== 1:
        LinearMotor.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)
    turnOffMotors()
    print ("Platform Centered")

def reset_linear_platform():
    if detect(endstop_pins[0])== 1:
        print("At endstop 1")
        turnOffMotors()
    while detect(endstop_pins[0])==0:
        LinearMotor.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)
    while detect(endstop_pins[0])==1:
        LinearMotor.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.INTERLEAVE)
    turnOffMotors()
    print ("linear platform reset")

#takes a picture with Camera 1 and stores
#the .jpg in "file" (path), then takes a picture with Camera 2
#and does the same
def take_picture_cameras(file):
    GPIO.output(camera_pins[0],False)
    GPIO.output(camera_pins[1],False)
    GPIO.output(camera_pins[2],True)
    timestr1 = time.strftime("/C1_D%Y_%m_%d-T%H_%M_%S")
    #make_dir=time.strftime("Camera1_D_%Y_%m_%d_T_%H_%M")
    capture1="raspistill -t 500 -st -o %s.jpg" % (file+timestr1)
    os.system(capture1)
    print("Camera 1 Capture Succesful")
    GPIO.output(camera_pins[0],True)
    GPIO.output(camera_pins[1],False)
    GPIO.output(camera_pins[2],True)
    timestr2 = time.strftime("/C2_D%Y_%m_%d-T%H_%M_%S")
    capture2 = "raspistill -t 500 -st -o %s.jpg" % (file+timestr2)
    os.system(capture2)
    print("Camera 2 Capture Successful")

def rotate_steps(n_steps):
    for i in range(int(n_steps)):
        RotaryMotor.oneStep(Adafruit_MotorHAT.FORWARD,Adafruit_MotorHAT.MICROSTEP)

def rotate_vial(interval):
    rotate_steps(interval)
    turnOffMotors()
    print ("vial rotated by", interval)

def rotate_reset():
    for i in range(int(1600)):
        RotaryMotor.oneStep(Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.MICROSTEP)
    print("Vial Rotated to Reset")

## ------------------- ##

def input_cmd(): ## returns object with parameters
    n_vials = int(input("Number of Vials (6 or less): "))
    steps = int(input("Number of pictures per vial: "))
    time = float(input("Length of Operation (mins): "))
    n_reps = int(input("Number of repetitions in length of operation: "))
    #store_path = str(raw_input("Path (folder in home dir): "))

    return ob.Operation_Input(n_vials, steps, time, n_reps)
def root_file_gen(root_file_name):
    bin_path = "/home/pi/"
    newpath = bin_path + root_file_name
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    
    return (newpath)

def run_file_gen(rootpath):
    run_folder = time.strftime("Run_%Y_%m_%d-%H:%M")
    newpath = rootpath+run_folder
    if not os.path.exists(newpath):
        os.makedirs(newpath)

    return (newpath)

def vial_file_gen(rootpath, n_vial):
    vial_folder = ("Vial_" + str(n_vial))
    rootpath = os.path.join(rootpath, vial_folder)
    if not os.path.exists(rootpath):
        os.makedirs(rootpath)

    return (rootpath)

def routine_file_gen(vialpath, Trial_Count):
    routine_folder = ("Trial_" + str(Trial_Count))
    vialpath = os.path.join(vialpath, routine_folder)
    if not os.path.exists(vialpath):
        os.makedirs(vialpath)
    print("Saved")
    return (vialpath)


def operation(input_ob,rootpath):
    n_vials = input_ob.n_vials
    steps = input_ob.steps
    time = input_ob.time
    n_reps = input_ob.n_reps
    trial_path=rootpath + "/Trial_Count.txt"
    trial_count= read_trial_count(trial_path)
    interval = math.floor(1600 / steps)
    remainder = 1600 - (steps * interval)
    reset_linear_platform()
    for i in range(n_vials):
        vialpath = vial_file_gen(rootpath, i+1)    
        next_endstop_number=2+i
        trial_folder = routine_file_gen(vialpath, trial_count)
        for j in range(steps):
            turnOnLights()
            take_picture_cameras(trial_folder)
            rotate_vial(interval)
        turnOffLights()
        if n_vials == 1:
            move_linear_platform(next_endstop_number)
            reset_linear_platform()
        elif i == n_vials-1:
            reset_linear_platform()
        else:
            move_linear_platform(next_endstop_number)
        for k in range(int(remainder)):
            RotaryMotor.oneStep(Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.MICROSTEP)
        rotate_reset()
    inc_trial_count(trial_path)
    print ("rep complete")

def main():
    input_ob = input_cmd()
    rootpath = root_file_gen(Root_File_Name)
    runpath = run_file_gen(rootpath)
    Trial_Count= initial_trial_count(runpath)
    periodic_scheduler = ob.Periodic_Scheduler(input_ob)
    periodic_scheduler.setup(60.0 * input_ob.time / float(input_ob.n_reps), operation, (input_ob, runpath))
    periodic_scheduler.run()

if __name__ == '__main__':
    main()