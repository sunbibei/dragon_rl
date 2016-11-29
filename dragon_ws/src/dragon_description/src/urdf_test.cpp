#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <iostream>


int main(int argc, char const *argv[]) {

  urdf::Model robot_model;
    // TODO: by parameter?
    robot_model.initFile("/home/w/catkin_ws/src/dragon_description/urdf/dragon.urdf");
    std::cout<<"frgse"<<std::endl;
   

    // Create active arm chain.
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)){
      std::cout<<"Could not convert urdf into kdl tree"<<std::endl;
      return -1;
    }
    std::cout<<"frgse"<<std::endl;
  
  return 0;
}
