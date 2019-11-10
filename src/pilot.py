#!/usr/bin/env python

import io
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
# converter = tf.contrib.lite.TFLiteConverter.from_keras_model(model)
# tflite_model = converter.convert()
graph = tf.get_default_graph()
pub_act_ctl = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)

def img_callback(img_msg):
    
    # publish actuator steering with inference result if in OFFBOARD MODE
    global offboard_mode_is_active
    if offboard_mode_is_active:
        global model
        global graph
        global pub_act_ctl
        # preprocess img as in ML pipeline with PIL
        # img = np.array(Image.frombytes('RGB',(160,120),io.BytesIO(img_msg.data),'raw'))
        img = np.array(Image.open(io.BytesIO(img_msg.data)))
        # img_arr = np.frombuffer(img_msg.data, dtype='int8').reshape(160, 120, 3)
        # img inference
        
        #deb = rospy.Time.now()
        
        with graph.as_default():
            #probs = model.predict(img_arr.reshape((1,) + img_arr.shape))
            probs = model.predict(img.reshape((1,) + img.shape))

        #print(rospy.Time.now()-deb)

        # process classes to output [-1,1] from 15 classes array
        bucket_no = np.argmax(probs)
        act_val = (bucket_no / 7) - 1.
        # publish value in actuator_output_msg.channels[1] and fix power in actuator_output_msg.channels[1]
        actuator_control = ActuatorControl()
        actuator_control.header.stamp = rospy.Time.now()
        actuator_control.group_mix = 0
        actuator_control.controls[0] = 0.0
        actuator_control.controls[1] = 0.0
        actuator_control.controls[2] = act_val
        actuator_control.controls[3] = 0.2       # Valeur entre 0 et 1 
        actuator_control.controls[4] = 0.0
        actuator_control.controls[5] = 0.0
        actuator_control.controls[6] = 0.0
        actuator_control.controls[7] = 0.0

        pub_act_ctl.publish(actuator_control)

def rcin_callback(rcin_msg):

    global offboard_mode_is_active
    
    # pilot
    # channel 6
    # [1750 2000[ : switch INFERENCE MODE on
    if int(rcin_msg.channels[5]) > 1750:
        offboard_mode_is_active = True
    else:
        offboard_mode_is_active = False


def main():

    rospy.init_node('pilot', anonymous=True)
   
    rospy.Subscriber("mavros/rc/in", RCIn , rcin_callback)

    rospy.Subscriber("raspicam_node/image/compressed", CompressedImage , img_callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()

