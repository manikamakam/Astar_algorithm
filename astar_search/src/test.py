
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def ros_inputs(Xi, Yi, Thetai, UL, UR):
    r = 3.8
    L = 35.4
    dt = 2
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    Xn = round(r * (UL + UR) * math.cos(Thetan) * dt)
    Yn = round(r * (UL + UR) * math.sin(Thetan) * dt)
    dTheta = (r / L) * (UR - UL) * dt


    dTheta = round(180 * (dTheta) / 3.14)
    vel_mag = math.sqrt(Xn** 2 + Yn** 2)


    return vel_mag, dTheta



rospy.init_node('astar_search', anonymous=True)
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
msg = Twist()

turtlebot3_model = rospy.get_param("model", "burger")


msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

velPub.publish(msg)

c =0
r = rospy.Rate(10)


while not rospy.is_shutdown():
    if c== 20:
        msg.linear.x = 0
        msg.angular.z = 0
        velPub.publish(msg)
    if c < 20:
        vel , th = ros_inputs(0,0,0,10,10)
        msg.linear.x = vel
        msg.angular.z = th 
        velPub.publish(msg)
        c=c+1
        r.sleep()










  # for (auto motion : steps) {
  #   /// defining ur, ul and  thetadot based on differential drive model
  #   double ur = (motion.at(0) * M_PI) / 30;
  #   double ul = (motion.at(1) * M_PI) / 30;
  #   double thetadot = (r / l) * (ur - ul);
  #   /// calculating change in angle
  #   double dtheta = thetadot * dt;
  #   double change_theta = theta + dtheta;

  #   double xdot = (r / 2) * (ul + ur) * cos(change_theta);
  #   double ydot = (r / 2) * (ul + ur) * sin(change_theta);
  #   /// setting the dx for node
  #   double dx = round(xdot * dt);
  #   /// setting the dy for node
  #   double dy = round(ydot * dt);
  #   /// setting the cost
  #   double cost = std::sqrt(std::pow((dx), 2) + std::pow((dy), 2));
  #   /// setting velocity magnitude
  #   double vel_mag = std::sqrt(std::pow(xdot, 2) + std::pow(ydot, 2));

  #   action.push_back({ dx, dy, cost, change_theta, vel_mag, dtheta });
  