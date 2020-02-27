/*my includes*/
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

/*gazebo plugin includes*/
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

ros::Publisher sensor_pub;
ros::Subscriber wheel_com_sub;

double encoder_ticks_per_rev;
double max_motor_rot_vel;
double max_motor_power;
double left_wheel_vel = 0;
double right_wheel_vel = 0;
int left_encoder = 0;
int right_encoder = 0;
double freq;
double curr_time;
double prev_time;

std::string left_wheel_joint;
std::string right_wheel_joint;
std::string wheel_cmd_tpc;
std::string sensor_data_tpc;

void wheel_com_sub_callback(nuturtlebot::WheelCommands data)
{
  left_wheel_vel = ((double)data.left_velocity)/((double) max_motor_power) * max_motor_rot_vel;
  right_wheel_vel = ((double)data.right_velocity)/((double) max_motor_power) * max_motor_rot_vel;
  // ROS_INFO("%f %f", left_wheel_vel, right_wheel_vel);
  return;
}


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("Hello World!");
      ros::NodeHandle nh;
      nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
      nh.getParam("/motor/max_power", max_motor_power);
      nh.getParam("/motor/no_load_speed", max_motor_rot_vel);


      freq = _sdf->GetElement("sensor_frequency")->Get<double>();
      left_wheel_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
      right_wheel_joint = _sdf->GetElement("right_wheel_joint")->Get<std::string>();
      wheel_cmd_tpc = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>(); 
      sensor_data_tpc = _sdf->GetElement("sensor_data_topic")->Get<std::string>(); 

      sensor_pub = nh.advertise<nuturtlebot::SensorData>(sensor_data_tpc, 1);
      wheel_com_sub = nh.subscribe(wheel_cmd_tpc, 1, &wheel_com_sub_callback);

      curr_time = ros::Time::now().toSec();
      prev_time = curr_time;

      ROS_INFO("%f", curr_time);
      ROS_INFO_STREAM(left_wheel_joint);

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, 1000.0);
      this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0 , left_wheel_vel);
      this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, 1000.0);
      this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0 , right_wheel_vel);

      curr_time = ros::Time::now().toSec();
      if ((1.0/(curr_time - prev_time)) < freq)
      {
        prev_time = curr_time;
        nuturtlebot::SensorData msg;
        left_encoder += (int)((left_wheel_vel * (1/freq))/(2 * rigid2d::PI) * encoder_ticks_per_rev);
        right_encoder += (int)((right_wheel_vel * (1/freq))/(2 * rigid2d::PI) * encoder_ticks_per_rev);
        msg.left_encoder = left_encoder;
        msg.right_encoder = right_encoder;
        sensor_pub.publish(msg);
        ROS_INFO("%d %d",msg.left_encoder,  msg.right_encoder);
      }


    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush) //michael said move this somewhere

}