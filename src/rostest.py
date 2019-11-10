#!/usr/bin/env python
import os
import rospy
import geometry_msgs.msg
import tensorflow as tf
import message_filters
from pathlib import Path
# from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import RCOut


# steering = 1500

def process_combined(image_msg, actuator_output_msg):
    steering = actuator_output_msg.channels[1]
    filename = '/tmp/oli/{0}-{1}-{2}'.format(str(image_msg.header.stamp.secs), str(image_msg.header.stamp.nsecs),str(steering))
    with open(filename, "wb")  as outfile:
        outfile.write(image_msg.data)

def main():

    rospy.init_node('rostester', anonymous=True)

    # sub_image = rospy.Subscriber("raspicam_node/image", Image, process_image, queue_size=1)
    # sub_outputs = rospy.Subscriber("mavros/rc/out", RCOut, process_actuator_outputs, queue_size=1)

    sub_image = message_filters.Subscriber('raspicam_node/image', Image)
    sub_outputs = message_filters.Subscriber('mavros/rc/out', RCOut)

    ts = message_filters.ApproximateTimeSynchronizer([(sub_image,), (sub_outputs,)], 1, 15000, allow_headerless=False)

    ts.registerCallback(process_combined)

    # print("getting reference to ROS topic")
    # pub = rospy.Publisher('/olitest', PoseStamped, queue_size=10)


    # print("initiating node rostester")
    # rate = rospy.Rate(10)  # 10hz

    # print("entering loop")
    # tf.enable_eager_execution()
    # print(tf.reduce_sum(tf.random_normal([1000, 1000])))
    # print("olidne")

    rospy.spin()
    # while not rospy.is_shutdown():

    #   rospy.spinonce()

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

def test():
# Récupérer une image sans header avec Pil
img_20889.jpg
from PIL import Image
infile = open('img_2859.txt','rb')
data = infile.read()
infile.close()
image=Image.frombytes('RGB',(160,120),data,'raw')
image1 = image.convert('LA')
image1.save('greyscale.png')

# permet de convertir la PIL.Image.Image en numpy.ndarray qui peut ensuite être utilisé dans opencv
# img=cv2.cvtColor(pix, cv2.COLOR_BGR2GRAY)
pix=np.array(image) 
# pic = image
# Then, after you make your changes to the array, you should be able to do either pic.putdata(pix) 
# or create a new image with Image.fromarray(pix).

# switch R & B color channels
r, g, b = im_rgb.split()
im_rgb = Image.merge('RGB', (b, g, r))


# numpy array
import numpy as np
nparr = np.frombuffer(data, np.uint8)

    
# image brute et affichage avec pyplot
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np


# Parameters.
input_filename = "img_2896.txt"
shape = (480, 128) # matrix size
dtype = np.dtype('<u1') # little-endian unsigned integer 8 bit # np.dtype('>u2') # big-endian unsigned integer (16bit)
output_filename = "rien.PNG"

# Reading.
fid = open(input_filename, 'rb')
data = np.fromfile(fid, dtype)
image = data.reshape(shape)

# Display.
plt.imshow(image, cmap = "gray")
plt.savefig(output_filename)
plt.show()

if __name__ == "__main__":
    main()
