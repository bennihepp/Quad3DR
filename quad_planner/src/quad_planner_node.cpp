#include <boost/thread.hpp>
#include <ros/ros.h>

#include <quad_planner/quad_planner.h>
#include <quad_planner/rendering/visualizer.h>


namespace quad_planner
{

class QuadPlannerNode
{
private:
  ros::NodeHandle node_handle_;

  QuadPlanner *quad_planner_ptr_;
  double octomap_resolution;

public:
  QuadPlannerNode()
  : node_handle_("~")
  {
    std::string octomap_filename;
    node_handle_.getParam("gazebo_octomap", octomap_filename);
    node_handle_.param("octomap_resolution", octomap_resolution, 0.1);

    quad_planner_ptr_ = new QuadPlanner(octomap_resolution);
    if (octomap_filename.empty())
    {
      octomap_filename = "/home/bhepp/Projects/Quad3DR/gazebo_octomap.bt";
    }
    if (!octomap_filename.empty())
    {
      quad_planner_ptr_->loadOctomapFile(octomap_filename);
    }
  }

  ~QuadPlannerNode()
  {
    delete quad_planner_ptr_;
  }

  const QuadPlanner& getQuadPlanner() const
  {
    return *quad_planner_ptr_;
  }

  QuadPlanner& getQuadPlanner()
  {
    return *quad_planner_ptr_;
  }

  void run()
  {
    quad_planner_ptr_->run();
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_planner");

  quad_planner::QuadPlannerNode node;
  quad_planner::rendering::Visualizer window;
  window.init();
  window.getOctomapRenderer()->setOctomap(node.getQuadPlanner().getOctomap());
  window.run();
//  node.run();

  return 0;
}

