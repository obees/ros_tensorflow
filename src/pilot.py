#!/usr/bin/env python
import os
import rospy
import geometry_msgs.msg
import tensorflow as tf
import numpy as np
import message_filters
import keras
from pathlib import Path
from PIL import Image
# from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import ActuatorControl
from sensor_msgs.msg import CompressedImage

# steering = 1500
offboard_mode_is_active = False
model_file_base = "/home/spider-n2/robocar/data/processed/model_keras"
model = keras.models.load_model(model_file_base)
pub_act_ctl = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

def process_combined(image_msg, actuator_output_msg):
    steering = actuator_output_msg.channels[1]
    filename = '/tmp/oli/{0}-{1}-{2}'.format(str(image_msg.header.stamp.secs), str(image_msg.header.stamp.nsecs),str(steering))
    with open(filename, "wb")  as outfile:
        outfile.write(image_msg.data)

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
        offboard_mode_is_active =True


def main():

    rospy.init_node('pilot', anonymous=True)
   
    # sub_image = rospy.Subscriber("raspicam_node/image", Image, process_image, queue_size=1)
    # sub_outputs = rospy.Subscriber("mavros/rc/out", RCOut, process_actuator_outputs, queue_size=1)
    rospy.Subscriber("mavros/rc/in", RCIn , rcin_callback)

    rospy.Subscriber("raspicam_node/image/compressed", CompressedImage , img_callback)


    # sub_image = message_filters.Subscriber('raspicam_node/image', Image)
    # sub_outputs = message_filters.Subscriber('mavros/rc/out', RCOut)
  
    # ts = message_filters.ApproximateTimeSynchronizer([(sub_image,), (sub_outputs,)], 1, 15000, allow_headerless=False)
  
    # ts.registerCallback(process_combined)

    # print("getting reference to ROS topic")
    # pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)


    # print("initiating node rostester")
    # rate = rospy.Rate(10)  # 10hz

    # print("entering loop")
    # tf.enable_eager_execution()
    # print(tf.reduce_sum(tf.random_normal([1000, 1000])))
    # print("olidne")
    
    rospy.spin()
    # while not rospy.is_shutdown():

    #	rospy.spinonce()

    #s    rate.sleep()

def process_actuator_outputs(actuator_output_msg):
    steering = actuator_output_msg.channels[1]
    filename = '/tmp/oli/o{0}-{1}-{2}'.format(str(actuator_output_msg.header.stamp.secs), str(actuator_output_msg.header.stamp.nsecs),str(steering))
    Path(filename).touch()

def process_image(image_msg):
    # steering = 1500
    # filename = '/tmp/oli/i{0}-{1}'.format(str(image_msg.header.stamp.secs), str(image_msg.header.stamp.nsecs))
    now = rospy.get_rostime()
    filename = '/tmp/oli/i{0}-{1}'.format(str(now.secs),str(now.nsecs))
    # ros::time::now()
    with open(filename, "wb")  as outfile: 
        # while True:
        #     buf=image_msg.data.read(1024)
        #     if buf: 
        #         outfile.write(buf)
        #     else:
        #         break 
        outfile.write(image_msg.data) 

if __name__ == "__main__":
    main()

