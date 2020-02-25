#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
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

void wheel_com_sub_callback(nuturtlebot::WheelCommands data)
{

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
      sensor_pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 1);
      wheel_com_sub = nh.subscribe("wheel_cmd", 1, &wheel_com_sub_callback);
      nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
      nh.getParam("/motor/max_power", max_motor_power);
      nh.getParam("/motor/no_load_speed", max_motor_rot_vel);

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
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush) //michael said move this somewhere
}