#include <ros/ros.h>

#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

#include <eigen_conversions/eigen_msg.h>

class DrivingAidMarker
{
public:
  enum Markers
  {
    LEFT_WHEEL_TRACE,
    RIGHT_WHEEL_TRACE,
    LEFT_FRONT_WHEEL,
    RIGHT_FRONT_WHEEL,
    LEFT_REAR_WHEEL,
    RIGHT_REAR_WHEEL
  };


  DrivingAidMarker()
  {

    ros::NodeHandle pnh("~");

    pnh.param("left_side_y_outer", p_left_side_y_outer_, 0.0);
    pnh.param("left_side_y_inner", p_left_side_y_inner_, 0.0);
    pnh.param("right_side_y_outer", p_right_side_y_outer_, 0.0);
    pnh.param("right_side_y_inner", p_right_side_y_inner_, 0.0);
    pnh.param("offset_z", p_offset_z_, -0.112-0.07);

    pnh.param("wheel_track", p_wheel_track_, 1.0);
    pnh.param("wheel_base", p_wheel_base_, 2.0);
    pnh.param("wheel_radius", p_wheel_radius_, 0.3);
    pnh.param("wheel_width", p_wheel_width_, 0.2);


    //vehicle frame is between rear axles projected to ground
    pnh.param("vehicle_frame", p_vehicle_frame_, std::string("vehicle_frame"));



    //pub_timer_ = pnh.createTimer(ros::Duration(0.1), &DrivingAidMarker::pubTimerCallback, this, false);
    marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("driving_aid", 1,false);

    steering_angle_sub_ = pnh.subscribe("steering_angle", 5, &DrivingAidMarker::steeringAngleCallback, this);

    visualization_msgs::Marker marker;
    //marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = p_vehicle_frame_;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r= 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.ns ="wheel_footprint";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    /*
    std::vector<geometry_msgs::Point> point_vector;

    geometry_msgs::Point tmp;
    tmp.x = 0.0;
    tmp.z = p_offset_z_;
    tmp.y = p_left_side_y_outer_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_left_side_y_inner_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_right_side_y_outer_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);

    tmp.x = 0.0;
    tmp.y = p_right_side_y_inner_;
    point_vector.push_back(tmp);
    tmp.x = 1.5;
    point_vector.push_back(tmp);


    marker.points = point_vector;
    */

    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(marker);

    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(marker);

    this->createWheelVis(marker_array_);


  }
  
  ~DrivingAidMarker()
  {
    
  }

  void steeringAngleCallback(const std_msgs::Float64& steering_angle)
  {
    generateVisualizationMarker(steering_angle.data, marker_array_);

    marker_pub_.publish(marker_array_);
  }

  void pubTimerCallback(const ros::TimerEvent& event)
  {
    if (marker_pub_.getNumSubscribers() > 0){
      marker_array_.markers[0].header.stamp = ros::Time::now();
      marker_pub_.publish(marker_array_);
    }
  }

  void generateVisualizationMarker(double steering_angle_rad, visualization_msgs::MarkerArray& marker_array)
  {
    double turn_radius = 0.0;

    if (std::abs(steering_angle_rad) > 0.000001){
      // use cotangent
      //turn_radius = std::tan((M_PI*0.5 - steering_angle_rad) * p_wheel_base_);
      turn_radius = (1.0/std::tan(steering_angle_rad)) * p_wheel_base_;
    }

    // (0,0) is rear axle middle
    // x axis towards front, y axis to left of car

    Eigen::Vector2d icc (0.0, turn_radius);

    Eigen::Vector2d left_wheel (p_wheel_base_, p_wheel_track_*0.5);
    Eigen::Vector2d right_wheel(p_wheel_base_, -p_wheel_track_*0.5);

    double dist_left  = (left_wheel  - icc).norm();
    double dist_right = (right_wheel - icc).norm();

    double steer_angle_left  = std::asin(p_wheel_base_/dist_left);
    double steer_angle_right = std::asin(p_wheel_base_/dist_right);

    if (turn_radius < 0){
      steer_angle_left  = -steer_angle_left;
      steer_angle_right = -steer_angle_right;
    }

    ROS_INFO("turn radius: %f dist left: %f right: %f", turn_radius, dist_left, dist_right);
    ROS_INFO("steer angle left: %f right: %f", steer_angle_left, steer_angle_right);

    marker_array.markers[0].points.resize(40);
    marker_array.markers[1].points.resize(40);


    std::vector<geometry_msgs::Point>& point_vector_left = marker_array.markers[0].points;
    std::vector<geometry_msgs::Point>& point_vector_right = marker_array.markers[1].points;

    marker_array.markers[1].color.r = 0.0;
    marker_array.markers[1].color.b = 1.0;
    marker_array.markers[1].id = 1;

    Eigen::Affine3d rot_left (Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX())*
                              Eigen::AngleAxisd(steer_angle_left, Eigen::Vector3d::UnitY()));

    tf::quaternionEigenToMsg(Eigen::Quaterniond(rot_left.rotation()), marker_array_.markers[LEFT_FRONT_WHEEL].pose.orientation);

    Eigen::Affine3d rot_right (Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX())*
                               Eigen::AngleAxisd(steer_angle_right, Eigen::Vector3d::UnitY()));

    tf::quaternionEigenToMsg(Eigen::Quaterniond(rot_right.rotation()), marker_array_.markers[RIGHT_FRONT_WHEEL].pose.orientation);


    //marker_array_.markers[LEFT_FRONT_WHEEL].pose.orientation; // = marker;
    //marker_array_.markers[RIGHT_FRONT_WHEEL].pose.orientation; // = marker;


    Eigen::Vector3d rotation_vector( Eigen::Vector3d::UnitZ() );

    if (turn_radius > 0.0){
      rotation_vector = -rotation_vector;
    }

    //std::cout << "rotation_vector:\n" << rotation_vector << "\n";

    for (size_t i = 0; i < 40; ++i)
    {
      Eigen::Affine3d o_t_i (Eigen::Affine3d::Identity());
      o_t_i.translation() = Eigen::Vector3d(icc.x(), -icc.y(), 0.0);

      //Eigen::Rotation2Dd rotation(steer_angle_left + static_cast<double>(i) * 0.05);



      Eigen::Affine3d rotation_left (Eigen::AngleAxisd(static_cast<double>(i) * 0.05,
                                     rotation_vector));

      //Eigen::Affine3d rotation_left (Eigen::AngleAxisd( static_cast<double>(i) * 0.05,
//                                     (turn_radius > 0.0) ? -(Eigen::Vector3d::UnitZ()) : Eigen::Vector3d::UnitZ() ));

      //Eigen::Vector2d tmp(o_t_i * rotation * left_wheel);
      //Eigen::Vector2d tmp(o_t_i * rotation * left_wheel).translation();
      Eigen::Vector3d tmp(o_t_i * rotation_left *Eigen::Vector3d(p_wheel_base_, (turn_radius)-p_wheel_track_*0.5, 0.0));

      point_vector_left[i].x = tmp.x();
      point_vector_left[i].y = -tmp.y();

      Eigen::Affine3d rotation_right (Eigen::AngleAxisd(static_cast<double>(i) * 0.05,
                                      rotation_vector));

      //Eigen::Affine3d rotation_right (Eigen::AngleAxisd( static_cast<double>(i) * 0.05,
      //                                 (turn_radius > 0.0) ? -(Eigen::Vector3d::UnitZ()) : Eigen::Vector3d::UnitZ() ));

      tmp = o_t_i * rotation_right*Eigen::Vector3d(p_wheel_base_, (turn_radius)+p_wheel_track_*0.5, 0.0);

      point_vector_right[i].x = tmp.x();
      point_vector_right[i].y = -tmp.y();
    }
  }

  void createWheelVis(visualization_msgs::MarkerArray& marker_array)
  {

    Eigen::Affine3d wheel_rot (Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX()));

    visualization_msgs::Marker marker;
    //marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = p_vehicle_frame_;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r= 1.0;
    marker.color.a = 1.0;
    marker.scale.x = p_wheel_radius_ * 2.0;
    marker.scale.y = p_wheel_radius_ * 2.0;
    marker.scale.z = p_wheel_width_;
    marker.ns ="wheels";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = p_wheel_radius_;

    tf::quaternionEigenToMsg(Eigen::Quaterniond(wheel_rot.rotation()), marker.pose.orientation);
    //marker.pose.orientation.w = 1.0;

    marker.id = LEFT_FRONT_WHEEL;
    marker.pose.position.x = p_wheel_base_;
    marker.pose.position.y = p_wheel_track_*0.5;
    marker_array_.markers[LEFT_FRONT_WHEEL] = marker;

    marker.id = RIGHT_FRONT_WHEEL;
    marker.pose.position.x =  p_wheel_base_;
    marker.pose.position.y = -p_wheel_track_*0.5;
    marker_array_.markers[RIGHT_FRONT_WHEEL] = marker;

    marker.id = LEFT_REAR_WHEEL;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = p_wheel_track_*0.5;
    marker_array_.markers[LEFT_REAR_WHEEL] = marker;

    marker.id = RIGHT_REAR_WHEEL;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = -p_wheel_track_*0.5;
    marker_array_.markers[RIGHT_REAR_WHEEL] = marker;


  }

protected:
  visualization_msgs::MarkerArray marker_array_;

  double p_left_side_y_outer_;
  double p_left_side_y_inner_;
  double p_right_side_y_outer_;
  double p_right_side_y_inner_;
  double p_offset_z_;

  double p_wheel_track_;
  double p_wheel_base_;
  double p_wheel_radius_;
  double p_wheel_width_;


  std::string p_vehicle_frame_;

  ros::Publisher marker_pub_;
  ros::Subscriber steering_angle_sub_;
  ros::Timer pub_timer_;
};

int main (int argc, char** argv)
{
  ros::init(argc,argv,"driving_aid_visualization_node");

  DrivingAidMarker driving_aid_marker;

  ros::spin();

  return (-1);
}
