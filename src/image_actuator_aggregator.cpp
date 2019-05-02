#include "ros/ros.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "img_act_aggregator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
// sub[i] = node.subscribe<sensor_msgs::JointState>(sub_name, 1, boost::bind(&Myclass::callback, this, _1, i));

  ros::Subscriber sub_image = nh.subscribe("raspicam_node/image", 1, process_image);
  ros::Subscriber sub_image = nh.subscribe("mavros/rc/out", 1, process_image);


   # sub_image = rospy.Subscriber("raspicam_node/image", Image, process_image, queue_size=1)
    # sub_outputs = rospy.Subscriber("mavros/rc/out", RCOut, process_actuator_outputs, queue_size=1)

// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%
  ros::Rate rate(30.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}