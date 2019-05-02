#!/usr/bin/env python

import rospy
import tensorflow as tf
import numpy as np
import keras
from PIL import Image
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import ActuatorControl
from sensor_msgs.msg import CompressedImage

offboard_mode_is_active = False
model_file_base = "/home/spider-n2/robocar/data/processed/model_keras"
model = keras.models.load_model(model_file_base)
pub_act_ctl = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)

def img_callback(img_msg):
    
    # publish actuator steering with inference result if in OFFBOARD MODE
    global offboard_mode_is_active
    if offboard_mode_is_active:
        global model
        global pub_act_ctl
        # preprocess img as in ML pipeline with PIL
        img = np.array(Image.frombytes('RGB',(160,120),img_msg.data,'raw'))
        
        # img inference
        probs = model.predict(img)

        # process classes to output [-1,1] from 15 classes array
        bucket_no = np.argmax(probs)
        act_val = (bucket_no / 7) - 1
        # publish value in actuator_output_msg.channels[1] and fix power in actuator_output_msg.channels[1]
        actuator_control = ActuatorControl()
        actuator_control.header.stamp = rospy.Time.now()
        actuator_control.group_mix = 0
        actuator_control.controls[0] = 0.0
        actuator_control.controls[1] = act_val
        actuator_control.controls[2] = 0.0
        actuator_control.controls[3] = -0.8
        actuator_control.controls[4] = 0.0
        actuator_control.controls[5] = 0.0
        actuator_control.controls[6] = 0.0
        actuator_control.controls[7] = 0.0

        pub_act_ctl.publish(actuator_control)

def rcin_callback(rcin_msg):

    global offboard_mode_is_active
    
    # pilot
    # channel 6
    # [1750 2000[ : switch OFFBOARD MODE on
    if int(rcin_msg.channels[5]) > 1750:
        offboard_mode_is_active = True


def main():

    rospy.init_node('pilot', anonymous=True)
   
    rospy.Subscriber("mavros/rc/in", RCIn , rcin_callback)

    rospy.Subscriber("raspicam_node/image/compressed", CompressedImage , img_callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()

