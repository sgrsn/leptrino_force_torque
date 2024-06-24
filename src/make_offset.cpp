#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class MakeOffset
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  double sum_fx_ = 0.0;
  double sum_fy_ = 0.0;
  double sum_fz_ = 0.0;
  double sum_tx_ = 0.0;
  double sum_ty_ = 0.0;
  double sum_tz_ = 0.0;
  int count_ = 0;

public:
  MakeOffset() : nh_("~")
  {
    sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("force_torque", 1, &MakeOffset::wrenchCallback, this);
  }

  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    // Sum the forces and torques
    sum_fx_ += msg->wrench.force.x;
    sum_fy_ += msg->wrench.force.y;
    sum_fz_ += msg->wrench.force.z;
    sum_tx_ += msg->wrench.torque.x;
    sum_ty_ += msg->wrench.torque.y;
    sum_tz_ += msg->wrench.torque.z;
    ++count_;
  }
  void saveData()
  {
    // Calculate averages and prepare for YAML saving
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    const double avg_fx = count_ > 0 ? sum_fx_ / count_ : 0.0;
    const double avg_fy = count_ > 0 ? sum_fy_ / count_ : 0.0;
    const double avg_fz = count_ > 0 ? sum_fz_ / count_ : 0.0;
    const double avg_tx = count_ > 0 ? sum_tx_ / count_ : 0.0;
    const double avg_ty = count_ > 0 ? sum_ty_ / count_ : 0.0;
    const double avg_tz = count_ > 0 ? sum_tz_ / count_ : 0.0;

    // Save to YAML
    out << YAML::Key << "offset_fx" << YAML::Value << avg_fx;
    out << YAML::Key << "offset_fy" << YAML::Value << avg_fy;
    out << YAML::Key << "offset_fz" << YAML::Value << avg_fz;
    out << YAML::Key << "offset_tx" << YAML::Value << avg_tx;
    out << YAML::Key << "offset_ty" << YAML::Value << avg_ty;
    out << YAML::Key << "offset_tz" << YAML::Value << avg_tz;
    out << YAML::EndMap;

    saveToYAML(out);
  }

  void saveToYAML(YAML::Emitter& out) 
  {
    std::string package_path = ros::package::getPath("leptrino_force_torque");
    // paramからfile名を取得
    std::string name = "";
    nh_.getParam("filename", name);
    std::string filename = package_path + "/config/" + name + ".yaml";

    std::ofstream fout(filename);
    if (!fout) {
      ROS_ERROR("Failed to open %s for writing.", filename.c_str());
      return;
    }
    fout << out.c_str();
    fout.close();
    if (fout) {
      ROS_INFO("Successfully wrote to %s.", filename.c_str());
    } else {
      ROS_ERROR("Failed to write to %s.", filename.c_str());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "wrench_offset_calculator");
  MakeOffset mo;
  ros::Rate rate(1200);
  ros::Time start_time = ros::Time::now();
  while (ros::ok())
  {
    ros::Duration elapsed = ros::Time::now() - start_time;
    if (elapsed.toSec() > 5.0)
    {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  mo.saveData();
  return 0;
}
