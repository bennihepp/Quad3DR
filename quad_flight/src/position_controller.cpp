// Adapted from hector_quadrotor package (https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor).

// Original Copyright note:
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <quad_flight/PositionControllerConfig.h>

namespace quad_flight
{

static std::ostream& operator<<(std::ostream& stream, const tf::Vector3 &vec)
{
  stream << "(" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << ")";
  return stream;
}

static std::ostream& operator<<(std::ostream& stream, const tf::Quaternion &quat)
{
  stream << "(" << quat.getX() << ", " << quat.getY() << ", " << quat.getZ() << ", " << quat.getW() << ")";
  return stream;
}

static std::ostream& operator<<(std::ostream& stream, const tf::Transform &transform)
{
  stream << "[origin: " << transform.getOrigin() << ", rotation: " << transform.getRotation() << "]";
  return stream;
}

static std::ostream& operator<<(std::ostream& stream, const tf::Matrix3x3 &matrix)
{
  stream << "(" << matrix.getRow(0).getX() << ", " << matrix.getRow(0).getY() << ", " << matrix.getRow(0).getZ() << std::endl
         << " " << matrix.getRow(1).getX() << ", " << matrix.getRow(1).getY() << ", " << matrix.getRow(1).getZ() << std::endl
         << " " << matrix.getRow(2).getX() << ", " << matrix.getRow(2).getY() << ", " << matrix.getRow(2).getZ() << ")" << std::endl;
  return stream;
}

class PositionController
{
private:
  ros::NodeHandle node_handle_;

  boost::thread loop_thread_;
  double control_rate_;

  // Transforms
  bool use_tf_;
  tf::TransformListener tf_listener_;
  std::string object_frame_;
  std::string world_frame_;
  double tf_wait_time_;

  // Subscribers and publishers
  ros::Publisher vel_cmd_publisher_;
  ros::Publisher vis_publisher_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber vel_subscriber_;
  ros::Subscriber pose_setpoin_subscriber_;

  dynamic_reconfigure::Server<quad_flight::PositionControllerConfig> reconfigure_server_;

  // Error integrals
  tf::Vector3 position_error_integral_;
  double yaw_error_integral_;

  // State
  tf::Transform pose_;
  tf::Transform pose_setpoint_;
  bool pose_valid_;
  bool pose_setpoint_valid_;

  // Parameters
  double gain_position_p_;
  double gain_orientation_p_;
  double gain_position_i_;
  double gain_orientation_i_;
  double position_error_integral_lim_;
  double orientation_error_integral_lim_;

  // Marker ID counter
  int marker_id_;

public:
  PositionController()
  {
    ros::NodeHandle params("~");

    params.param<double>("control_rate", control_rate_, 50);
    params.param<std::string>("world_frame", world_frame_, "world");
    params.param<std::string>("object_frame", object_frame_, "base_link");
    params.param<double>("tf_wait_time", tf_wait_time_, 10);
    params.param<bool>("use_tf", use_tf_, true);

    params.param<double>("gain_position_p", gain_position_p_, 1.0);
    params.param<double>("gain_orientation_p", gain_orientation_p_, 1.0);
    params.param<double>("gain_position_i", gain_position_i_, 0.002);
    params.param<double>("gain_orientation_i", gain_orientation_i_, 0.002);
    params.param<double>("position_error_integral_lim", position_error_integral_lim_, 2.0);
    params.param<double>("orientation_error_integral_lim", orientation_error_integral_lim_, 2.0);

    position_error_integral_.setZero();
    yaw_error_integral_ = 0;

    pose_setpoint_valid_ = false;

    marker_id_ = 0;

    if (!use_tf_) {
      pose_valid_ = false;
      pose_subscriber_ = node_handle_.subscribe<geometry_msgs::Pose>("pose", 1, boost::bind(&PositionController::poseCallback, this, _1));
    }
    pose_setpoin_subscriber_ = node_handle_.subscribe<geometry_msgs::Pose>("cmd_pose", 1, boost::bind(&PositionController::poseSetpointCallback, this, _1));
    vel_subscriber_ = node_handle_.subscribe<geometry_msgs::Twist>("teleop_vel", 1, boost::bind(&PositionController::teleopVelCallback, this, _1));

    vel_cmd_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    vis_publisher_ = node_handle_.advertise<visualization_msgs::Marker>( "pose_setpoint_marker", 10);

    dynamic_reconfigure::Server<PositionControllerConfig>::CallbackType f;
    f = boost::bind(&PositionController::reconfigureCallback, this, _1, _2);
    reconfigure_server_.setCallback(f);
  }

  ~PositionController()
  {
    loop_thread_.join();
    stop();
  }

  void reconfigureCallback(quad_flight::PositionControllerConfig &config, uint32_t level) {
    gain_position_p_ = config.gain_position_p;
    gain_position_i_ = config.gain_position_i;
    gain_orientation_p_ = config.gain_orientation_p;
    gain_orientation_i_ = config.gain_orientation_i;
    position_error_integral_lim_ = config.position_error_integral_lim;
    orientation_error_integral_lim_ = config.orientation_error_integral_lim;
  }

  void poseCallback(const geometry_msgs::PoseConstPtr &pose)
  {
    pointMsgToVector3(pose->position, pose_.getOrigin());
    tf::Quaternion quat;
    quaternionMsgToTF(pose->orientation, quat);
    pose_.setRotation(quat);
    pose_valid_ = true;
  }

  void poseSetpointCallback(const geometry_msgs::PoseConstPtr &pose)
  {
    ROS_INFO_STREAM("Recevied pose setpoint message");
    pointMsgToVector3(pose->position, pose_setpoint_.getOrigin());
    tf::Quaternion quat;
    quaternionMsgToTF(pose->orientation, quat);
    pose_setpoint_.setRotation(quat);
    pose_setpoint_valid_ = true;
  }

  void teleopVelCallback(const geometry_msgs::TwistConstPtr &twist)
  {
    if (pose_setpoint_valid_)
    {
      tf::Vector3 linear_offset;
      vector3MsgToTF(twist->linear, linear_offset);
//       ROS_INFO_STREAM("linear_offset: " << linear_offset);
      pose_setpoint_.getOrigin() += linear_offset;
      tf::Vector3 rotation_axis;
      vector3MsgToTF(twist->angular, rotation_axis);
      double angle = rotation_axis.length();
      if (angle > 0) {
        rotation_axis /= angle;
        tf::Quaternion dquat(rotation_axis, angle);
//         ROS_INFO_STREAM("prev setpoint quaternion: " << pose_setpoint_.getRotation());
//         ROS_INFO_STREAM("dquat: " << dquat);
        pose_setpoint_.setRotation(pose_setpoint_.getRotation() * dquat);
//         ROS_INFO_STREAM("setpoint quaternion: " << pose_setpoint_.getRotation());
      }
    }
  }

  void stop()
  {
    if(vel_cmd_publisher_.getNumSubscribers() > 0)
    {
      geometry_msgs::Twist vel_cmd = geometry_msgs::Twist();
      vel_cmd_publisher_.publish(vel_cmd);
    }
  }

  void run()
  {
    // Waiting for valid time
    ROS_INFO_STREAM("Waiting for valid time...");
    while (ros::Time::now() == ros::Time())
    {
      ROS_INFO_STREAM(".");
      ros::Duration(0.1).sleep();
    }

    if (use_tf_)
    {
      try {
        ROS_INFO_STREAM("Waiting for transform from " << world_frame_ << " to " << object_frame_ << " (timeout=" << tf_wait_time_ << ")");
        tf_listener_.waitForTransform(world_frame_, object_frame_, ros::Time::now(), ros::Duration(tf_wait_time_));
        grabPoseSetpointFromTransform();
        pose_setpoint_ = pose_;
        pose_setpoint_valid_ = true;
        ROS_INFO_STREAM("Initial setpoint: " << pose_setpoint_);
      } catch (tf::TransformException &ex) {
        ROS_FATAL_STREAM("Error while waiting for transform: " << ex.what());
        ROS_INFO_STREAM("Available transform IDs:");
        std::vector<std::string> frameStrings;
        tf_listener_.getFrameStrings(frameStrings);
        for (auto &frameString : frameStrings) {
          ROS_INFO_STREAM("  id: " << frameString);
        }
        ros::shutdown();
      }
    }

    boost::thread thread([this]()
    {
      ros::Rate rate(control_rate_);
      while (ros::ok())
      {
        update();
        rate.sleep();
      }
    });
  }

  void grabPoseSetpointFromTransform()
  {
    tf::StampedTransform stamped_transform;
    tf_listener_.lookupTransform(world_frame_, object_frame_, ros::Time(0), stamped_transform);
    pose_ = stamped_transform;
  }

  void pointMsgToVector3(const geometry_msgs::Point &point, tf::Vector3 &tf_vec)
  {
    tf_vec.setX(point.x);
    tf_vec.setY(point.y);
    tf_vec.setZ(point.z);
  }

  void vector3TFToMsg(const tf::Vector3 &tf_vec, geometry_msgs::Vector3 &vec)
  {
    vec.x = tf_vec.getX();
    vec.y = tf_vec.getY();
    vec.z = tf_vec.getZ();
  }

  void vector3MsgToTF(const geometry_msgs::Vector3 &vec, tf::Vector3 &tf_vec)
  {
    tf_vec.setX(vec.x);
    tf_vec.setY(vec.y);
    tf_vec.setZ(vec.z);
  }

  void quaternionMsgToTF(const geometry_msgs::Quaternion &quat, tf::Quaternion &tf_quat)
  {
    tf_quat.setX(quat.x);
    tf_quat.setY(quat.y);
    tf_quat.setZ(quat.z);
    tf_quat.setW(quat.w);
  }

  void orientationTFtoMsg(const tf::Quaternion &tf_quat, geometry_msgs::Quaternion &quat)
  {
    quat.x = tf_quat.getX();
    quat.y = tf_quat.getY();
    quat.z = tf_quat.getZ();
    quat.w = tf_quat.getW();
  }

  double get_value_in_range(double value, double min, double max)
  {
    value = std::min(value, max);
    value = std::max(value, min);
    return value;
  }

  double get_value_in_range(double value, double abs_max)
  {
    return get_value_in_range(value, -abs_max, abs_max);
  }

  void update()
  {
//     ROS_INFO_STREAM("pose_setpoint: " << pose_setpoint_.getOrigin());
//     ROS_INFO_STREAM("pose: " << pose_.getOrigin());
    if (!pose_setpoint_valid_) {
      return;
    }
    if (use_tf_) {
      grabPoseSetpointFromTransform();
    } else {
      if (!pose_valid_) {
        return;
      }
    }

    tf::Vector3 position_error = pose_.getOrigin() - pose_setpoint_.getOrigin();
    position_error_integral_ += position_error;
    position_error_integral_.setX(get_value_in_range(position_error_integral_.getX(), position_error_integral_lim_));
    position_error_integral_.setY(get_value_in_range(position_error_integral_.getY(), position_error_integral_lim_));
    position_error_integral_.setZ(get_value_in_range(position_error_integral_.getZ(), position_error_integral_lim_));
//     ROS_INFO_STREAM("position_error_integral: " << position_error_integral_);

    tf::Vector3 linear_vel = -gain_position_p_ * position_error - gain_position_i_ * position_error_integral_;
//     Position error is in world coordinates so we have to translate the desired action to body frame
    linear_vel = pose_.getBasis().transpose() * linear_vel;

    // For debugging
//    ROS_INFO_STREAM("pose setp: " << pose_setpoint_);
//     ROS_INFO_STREAM("pose setp orientation: " << pose_setpoint_.getRotation());
//     ROS_INFO_STREAM("pose setp angle: " << pose_setpoint_.getRotation().getAngle());
//     ROS_INFO_STREAM("pose setp axis: " << pose_setpoint_.getRotation().getAxis());
//     double roll, pitch, yaw;
//     pose_setpoint_.getBasis().getRPY(roll, pitch, yaw);
//     ROS_INFO_STREAM("setp yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll);

    // For debugging
//    ROS_INFO_STREAM("pose: " << pose_);
//     ROS_INFO_STREAM("pose orientation: " << pose_.getRotation());
//     ROS_INFO_STREAM("pose angle: " << pose_.getRotation().getAngle());
//     ROS_INFO_STREAM("pose axis: " << pose_.getRotation().getAxis());
//     double roll, pitch, yaw;
//     pose_.getBasis().getRPY(roll, pitch, yaw);
//     ROS_INFO_STREAM("yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll);

    tf::Matrix3x3 orientation_error = pose_.getBasis() * pose_setpoint_.getBasis().transpose();
//     ROS_INFO_STREAM("orientation_error: " << orientation_error);
    double roll_error, pitch_error, yaw_error;
    orientation_error.getRPY(roll_error, pitch_error, yaw_error);
//     ROS_INFO_STREAM("error yaw=" << yaw_error << ", pitch=" << pitch_error << ", roll=" << roll_error);
    yaw_error_integral_ += yaw_error;
    yaw_error_integral_ = get_value_in_range(yaw_error_integral_, orientation_error_integral_lim_);
//     ROS_INFO_STREAM("yaw_error_integral: " << yaw_error_integral_);

    tf::Vector3 angular_vel;
    angular_vel.setX(0);
    angular_vel.setY(0);
    angular_vel.setZ(-gain_orientation_p_ * yaw_error - gain_orientation_i_ * yaw_error_integral_);

    geometry_msgs::Twist vel_cmd;
    vector3TFToMsg(linear_vel, vel_cmd.linear);
    vector3TFToMsg(angular_vel, vel_cmd.angular);
    vel_cmd_publisher_.publish(vel_cmd);
    publishVisualization();
  }

  void publishVisualization()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "pose_setpoint_marker";
    marker.id = marker_id_;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose_setpoint_.getOrigin().getX();
    marker.pose.position.y = pose_setpoint_.getOrigin().getY();
    marker.pose.position.z = pose_setpoint_.getOrigin().getZ();
    marker.pose.orientation.x = pose_setpoint_.getRotation().getX();
    marker.pose.orientation.y = pose_setpoint_.getRotation().getY();
    marker.pose.orientation.z = pose_setpoint_.getRotation().getZ();
    marker.pose.orientation.w = pose_setpoint_.getRotation().getW();
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.1);
    vis_publisher_.publish(marker);
    ++marker_id_;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_teleop");

  quad_flight::PositionController ctrl;
  ctrl.run();
  ros::spin();

  return 0;
}

