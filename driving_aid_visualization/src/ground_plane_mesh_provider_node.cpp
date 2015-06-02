#include <ros/ros.h>

#include <shape_msgs/Mesh.h>

class GroundPlaneMeshProvider
{
public:


  GroundPlaneMeshProvider()
  {

    ros::NodeHandle pnh("~");


    //vehicle frame is between rear axles projected to ground
    //pnh.param("vehicle_frame", p_vehicle_frame_, std::string("vehicle_frame"));



    //pub_timer_ = pnh.createTimer(ros::Duration(0.1), &DrivingAidMarker::pubTimerCallback, this, false);
    ground_plane_mesh_pub_ = pnh.advertise<shape_msgs::Mesh>("ground_plane_mesh", 1,true);

    shape_msgs::Mesh mesh;

    mesh.vertices.resize(4);

    mesh.vertices[0].x = -100.0;
    mesh.vertices[0].y = -100.0;
    mesh.vertices[0].z =  -0.04;

    mesh.vertices[1].x =  100.0;
    mesh.vertices[1].y = -100.0;
    mesh.vertices[1].z =  -0.04;

    mesh.vertices[2].x = 100.0;
    mesh.vertices[2].y = 100.0;
    mesh.vertices[2].z = -0.04;

    mesh.vertices[3].x = -100.0;
    mesh.vertices[3].y =  100.0;
    mesh.vertices[3].z =  -0.04;

    mesh.triangles.resize(2);

    mesh.triangles[0].vertex_indices[0] = 0;
    mesh.triangles[0].vertex_indices[1] = 1;
    mesh.triangles[0].vertex_indices[2] = 2;

    mesh.triangles[1].vertex_indices[0] = 2;
    mesh.triangles[1].vertex_indices[1] = 3;
    mesh.triangles[1].vertex_indices[2] = 0;

    ground_plane_mesh_pub_.publish(mesh);

  }
  


protected:
  /*
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
 */
  ros::Publisher ground_plane_mesh_pub_;
};

int main (int argc, char** argv)
{
  ros::init(argc,argv,"ground_plane_mesh_provider_node");

  GroundPlaneMeshProvider ground_plane_mesh_provider;

  ros::spin();

  return (-1);
}
