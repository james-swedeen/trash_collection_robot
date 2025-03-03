/**************************
 *
 * Gather Image Data
 * Search from 0 to Points width
 *  Get rid of all values that are beyond the arms reach (x,y,z) Meter??
 *  Push the remaining points to a vector
 * Search remaining Vector for specific color
 *  push remaining points to a second vector
 * Average x, y, z values of together to get the center of the object
 * return the x,y,z value
 * *************************/
#include<cstdint>
#include<mutex>
#include<vector>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include "ros/ros.h"
#include<Eigen/Dense>

struct Point
{
  float x;
  float y;
  float z;
  float junk;
  uint8_t blue;
  uint8_t green;
  uint8_t red;
  uint8_t alpha;
};

ros::Publisher pub;
std::mutex     pub_mux;
std::vector<std::vector<uint8_t>> ref_colors;

void pointCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  const size_t ref_colors_size = ref_colors.size();

/*  std::vector<uint64_t> average_count;
  std::vector<Eigen::Matrix<uint64_t,3,1>> average_color;*/
  std::vector<std::list<Eigen::Matrix<float,3,1>>> positions;
  positions.resize(ref_colors_size);
/*  average_color.resize(ref_colors_size);
  average_count.resize(ref_colors_size);
  for(size_t it = 0; it < 3; ++it)
  {
    average_color[it].setZero();
  }*/

  //Get data from pointcloud message and assign it to Point struct object
  Point *points = (Point*)(msg->data.data());
  //Get size of data
  const size_t point_cloud_size = msg->width - 1;

  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    for(size_t it = 0; it < point_cloud_size; ++it)
    {
      //const float real_x = points[it].z;
      //const float real_y = -points[it].x;
      const float real_z = -points[it].y;

      // Take care of any points larger than a meter
      if(0.1 > real_z)
      {
      //    if(0.01 > std::abs(points[it].y))
        //Calculate distance between reference colors and the points
        const double color_dist = std::pow(points[it].red   - ref_colors[color_it][0], 2) +
                                  std::pow(points[it].green - ref_colors[color_it][1], 2) +
                                  std::pow(points[it].blue  - ref_colors[color_it][2], 2);

        if(color_dist < std::pow(25, 2))         //Will want to check this later to make sure
        {
          positions[color_it].emplace_back();
          positions[color_it].back().x() = points[it].x;
          positions[color_it].back().y() = points[it].y;
          positions[color_it].back().z() = points[it].z;

  /*        average_color[color_it][0] += points[it].red;
          average_color[color_it][1] += points[it].green;
          average_color[color_it][2] += points[it].blue;
          average_count[color_it]++;*/
        }
      }
    }
  }

  std::vector<Eigen::Matrix<float,3,1>> averages;
  averages.resize(ref_colors_size);
  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    averages[color_it].setZero();
  }

  //Average Postions together to get center
  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    for(auto it = positions[color_it].cbegin(); it != positions[color_it].cend(); ++it)
    {
      averages[color_it].array() += it->array();
    }
  }

  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    if(not positions[color_it].empty())
    {
      averages[color_it].array() /= positions[color_it].size();
      positions[color_it].remove_if([&averages, color_it](const Eigen::Matrix<float,3,1>& current) -> bool
        {
          return ((averages[color_it].array() - current.array()).array().abs().array() > float(0.1)).any();
        });
    }
  }

  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    if(not positions[color_it].empty())
    {
      averages[color_it].setZero();

      for(auto it = positions[color_it].cbegin(); it != positions[color_it].cend(); ++it)
      {
        averages[color_it].array() += it->array();
      }

      averages[color_it].array() /= positions[color_it].size();

      std::cout << "After Removing Noise color " << color_it << " is at: " <<
                   "x postion: "   << averages[color_it].z() <<
                   "\ty position " << -averages[color_it].x() <<
                   "\tz position " << -averages[color_it].y() << std::endl;
      /*std::cout << "The color at that point is: " << average_color[color_it][0]/average_count[color_it] << " "
                                                  << average_color[color_it][1]/average_count[color_it] << " "
                                                  << average_color[color_it][2]/average_count[color_it] << std::endl;
      std::cout << average_count[color_it] << std::endl;*/
    }
  }

  for(size_t color_it = 0; color_it < ref_colors_size; ++color_it)
  {
    if(not positions[color_it].empty())
    {
      geometry_msgs::PoseStamped output_msg;
      output_msg.header = msg->header;
      output_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);

      const double mag = averages[color_it].norm();

      output_msg.pose.position.x = averages[color_it].x() + (double(0.01) * (averages[color_it].x() / mag));
      output_msg.pose.position.y = averages[color_it].y() + (double(0.01) * (averages[color_it].y() / mag));
      output_msg.pose.position.z = averages[color_it].z() + (double(0.01) * (averages[color_it].z() / mag));

      pub_mux.lock();
      pub.publish(output_msg);
      pub_mux.unlock();
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc,argv, "Main");
  ros::NodeHandle nh;

  // Add colors
  ref_colors.emplace_back(std::vector<uint8_t>({0,115,196})); // Blue
  ref_colors.emplace_back(std::vector<uint8_t>({190,42,40})); // Red
  ref_colors.emplace_back(std::vector<uint8_t>({13,150,113})); // Green

  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 5, pointCallBack);
  pub = nh.advertise<geometry_msgs::PoseStamped>("targets", 10);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}
