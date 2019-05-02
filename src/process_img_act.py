#!/usr/bin/env python

import rosbag
import yaml
import datetime
import os
import json

# bagfile_no = '4'
# bagfile_name = '/home/spider-n2/bagfiles/subset' + bagfile_no + '.bag'
img_bagfile = '/home/spider-n2/bagfiles/img.bag'
tac_bagfile = '/home/spider-n2/bagfiles/tac.bag'

timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
folder = '/robocar/data/processed/' + timestamp
folder_full_path = '/home/spider-n2' + folder + '/'
os.mkdir(folder_full_path)

events_folder = '/events'
event_full_path = '/home/spider-n2' + folder + '/events'
os.mkdir(event_full_path)

jpg_folder = '/jpg' 
jpg_full_path = '/home/spider-n2' + folder + '/jpg'
os.mkdir(jpg_full_path)

filename = '/home/spider-n2' + folder + events_folder + '/actuators_output_msgs_json.txt'
with open(filename, "wb")  as outfile:
        # while True:
        #     buf=image_msg.data.read(1024)
        #     if buf:
        #         outfile.write(buf)
        #     else:
        #         break
    outfile.write('{"actuator_events":[') 
    for topic, msg, t in rosbag.Bag(tac_bagfile).read_messages():
        #if topic == "/mavros/rc/out":
        if topic == "/mavros/target_actuator_control":
            msg_json = yaml.safe_load(str(msg))
            outfile.write(json.dumps(msg_json)+',')
            # outfile.write(str(msg)+',')
    outfile.seek(-1,os.SEEK_END)
    outfile.write(']}')


filename = '/home/spider-n2' + folder + events_folder + '/images_jpg_msgs_json.txt'
with open(filename, "wb")  as outfile:
    outfile.write('{"images_jpg_events":[')
    for topic, msg, t in rosbag.Bag(img_bagfile).read_messages():
        if topic == "/raspicam_node/image/compressed":
            filename2 = '/home/spider-n2' + folder + jpg_folder + '/img_'+str(msg.header.seq)+'.jpg'
            with open(filename2, "wb")  as outfile2:
                #mdata=' '.join(format(ord(i),'b').zfill(8) for i in msg.data)
                #mdata=bytearray(msg.data)
                #print(''.join(format(ord(i),'b').zfill(8) for i in msg.data[0]))
                #while True:
                #    buf=mdata.read(1024)
                #    if buf:
                #        outfile2.write(buf)
                #    else:
                #        break
                #print(msg.data)
                outfile2.write(msg.data)
                #print(msg.data)
            msg.data=None
            msg_json = yaml.safe_load(str(msg))
            outfile.write(json.dumps(msg_json)+',')
            # outfile.write('- ' +str(msg)+'\n')

    outfile.seek(-1,os.SEEK_END)
    outfile.write(']}')
###with open(filename, 'r') as stream:
###    try:
###        print(yaml.load(stream))
###    except yaml.YAMLError as exc:
###        print(exc)
#for topic, msg, t in rosbag.Bag('/home/spider-n2/bagfiles/subset.bag').read_messages():
#    if topic == "/mavros/rc/out":
#        # boucle pour mettre  jour le timestamp du rc out
#        
#    #    if abs(msg.header.stamp - estimated_offset_ns) < 33000000:
## + remote_timestamp_ns
#        print("out"+str(msg.header.stamp))
#        print("t"+str(t))
#    elif topic == "/raspicam_node/image":
#        print("img"+str(msg.header.stamp))    
#    elif topic == "/mavros/timesync_status":
#        print("tss"+str(msg.header.stamp))
