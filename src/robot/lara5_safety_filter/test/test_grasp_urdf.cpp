// Static collision-model preparation tests; no ROS nodes or physics.
#include "lara5_safety_filter/urdf_preprocessor.hpp"
#include <tinyxml2.h>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace lara5_safety_filter;

void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

Eigen::Matrix3d rotation(tinyxml2::XMLElement* joint)
{
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  auto* origin = joint->FirstChildElement("origin");
  if (origin && origin->Attribute("rpy"))
  {
    std::istringstream stream(origin->Attribute("rpy"));
    stream >> rpy.x() >> rpy.y() >> rpy.z();
  }
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

int main()
{
  try
  {
    const std::string urdf = R"(<robot name="test"><link name="base"/><link name="flange"/>
      <link name="finger"/><link name="tip"/>
      <joint name="arm" type="revolute"><parent link="base"/><child link="flange"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
      <joint name="finger_joint" type="revolute"><parent link="flange"/><child link="finger"/>
        <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="2" effort="1" velocity="1"/></joint>
      <joint name="tip_joint" type="revolute"><parent link="finger"/><child link="tip"/>
        <origin xyz="0 0 0.1" rpy="0.2 0 0"/><axis xyz="0 1 0"/>
        <limit lower="0" upper="2" effort="1" velocity="1"/>
        <mimic joint="finger_joint" multiplier="1.2" offset="0.1"/></joint>
      </robot>)";
    tinyxml2::XMLDocument doc;
    doc.Parse(prepareUrdf(urdf, {"arm"}, {{"finger_joint", 0.5}}).c_str());
    auto* joint = doc.FirstChildElement("robot")->FirstChildElement("joint");
    require(std::string(joint->Attribute("type")) == "revolute", "Arm must remain movable");
    joint = joint->NextSiblingElement("joint");
    require(std::string(joint->Attribute("type")) == "fixed", "Finger must become fixed");
    require(rotation(joint).isApprox(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix(), 1e-12),
            "Frozen finger must retain its actual angle");
    joint = joint->NextSiblingElement("joint");
    const Eigen::Matrix3d expected = (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY())).toRotationMatrix();
    require(rotation(joint).isApprox(expected, 1e-12), "Mimic rotation must compose after the original origin");
    require(!joint->FirstChildElement("mimic"), "Prepared fixed model must have no mimic tags");
    const auto attached = attachPayloadBox(prepareUrdf(urdf, {"arm"}), "flange",
                                           {0.05, 0.02, 0.15}, {0, 0, 0}, {0.07, 0.07, 0.07});
    doc.Parse(attached.c_str());
    auto* payload = doc.FirstChildElement("robot")->LastChildElement("joint");
    require(std::string(payload->FirstChildElement("parent")->Attribute("link")) == "flange",
            "Payload must move with flange, not world");
    require(attached.find("grasp_payload_collision") != std::string::npos, "Carried object needs collision geometry");
    std::cout << "Frozen hand, mimic FK and flange-attached payload checks passed\n";
    return 0;
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
