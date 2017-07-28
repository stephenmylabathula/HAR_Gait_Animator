/*
 * SkeletonStructure.cpp
 * Created: July 28, 2017
 * Author: Stephen Mylabathula
 */

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <math.h>
#include "SkeletonStructure.h"

const float deg2rad = M_PI / 180;

SkeletonStructure::SkeletonStructure() {
  // initialize
  xyz_node_list = std::vector<XYZNode>(31);
}

SkeletonStructure::~SkeletonStructure() {
  // destroy
}

Eigen::Matrix3f SkeletonStructure::RotationMatrix(float xangle, float yangle, float zangle, std::string order) {
  if(order.empty())
    order = "zxy";

  float c1 = cos(xangle);
  float c2 = cos(yangle);
  float c3 = cos(zangle);
  float s1 = sin(xangle);
  float s2 = sin(yangle);
  float s3 = sin(zangle);

  Eigen::Matrix3f rotmat;

  if(order.compare("zxy") == 0)
    rotmat << c2*c3-s1*s2*s3, c2*s3+s1*s2*c3, -s2*c1, -c1*s3, c1*c3, s1, s2*c3+c2*s1*s3, s2*s3-c2*s1*c3, c2*c1;
  else {
    rotmat = rotmat.Identity();
    Eigen::Matrix3f temp;
    for (unsigned int i = 0; i < order.size(); i++)
      if(order.at(i) == 'x'){
        temp << 1, 0, 0, 0, c1, s1, 0, -s1, c1;
        rotmat = temp * rotmat;
      } else if(order.at(i) == 'y'){
        temp << c2, 0, -s2, 0, 1, 0, s2, 0, c2;
        rotmat = temp * rotmat;
      } else if(order.at(i) == 'z'){
        temp << c3, s3, 0, -s3, c3, 0, 0, 0, 1;
        rotmat = temp * rotmat;
      }
  }
  return rotmat;
}

std::vector<XYZNode> SkeletonStructure::GetChildXyz(std::vector<XYZNode> xyz_node_list, int ind, std::vector< std::vector<float> > channels) {
  int parent = this->tree[ind].parent;
  std::vector<int> children = this->tree[ind].children;
  float rotVal[] = {0, 0, 0};

  for (int i = 0; i < 3; i++){
    int rind = this->tree[ind].rotind[i];
    if(rind >= 0)
      rotVal[i] = channels[rind % channels.size()][(int)(rind / channels.size())];
    else
      rotVal[i] = 0;
  }

  Eigen::Matrix3f tdof = RotationMatrix(rotVal[0] * deg2rad, rotVal[1] * deg2rad, rotVal[2] * deg2rad, this->tree[ind].order);
  Eigen::Matrix3f torient = RotationMatrix(this->tree[ind].axis[0] * deg2rad, this->tree[ind].axis[1] * deg2rad, this->tree[ind].axis[2] * deg2rad, this->tree[ind].axis_order);
  Eigen::Matrix3f torientInv = torient.inverse();


  this->xyz_node_list[ind].rot = torientInv * tdof * torient * this->xyz_node_list[parent].rot;
  Eigen::Vector3f offset_as_vector;
  offset_as_vector << this->tree[ind].offset[0], this->tree[ind].offset[1], this->tree[ind].offset[2];
  Eigen::Vector3f xyz_as_vector;
  Eigen::MatrixXf mult = offset_as_vector.transpose() * this->xyz_node_list[ind].rot;

  this->xyz_node_list[ind].xyz[0] = this->xyz_node_list[parent].xyz[0] + mult(0);
  this->xyz_node_list[ind].xyz[1] = this->xyz_node_list[parent].xyz[1] + mult(1);
  this->xyz_node_list[ind].xyz[2] = this->xyz_node_list[parent].xyz[2] + mult(2);

  for (unsigned int i = 0; i < children.size(); i++){
    int cind = children[i];
    xyz_node_list = GetChildXyz(xyz_node_list, cind, channels);
  }

  return this->xyz_node_list;
}

std::vector< std::vector<float> > SkeletonStructure::ComputeSkeletonXYZPose(std::vector< std::vector<float> > channels) {
  std::vector<float> rotVal = this->tree[0].orientation;
  for (int i = 0; i < 3; i++) {
    int rind = this->tree[0].rotind[i];
    if (rind >= 0)
      rotVal[i] = rotVal[i] + channels[rind % channels.size()][(int)(rind / channels.size())];
  }

  Eigen::Matrix3f rotation_matrix = RotationMatrix(rotVal[0] * deg2rad, rotVal[1] * deg2rad, rotVal[2] * deg2rad, this->tree[0].axis_order);
  //XYZNode newNode;
  //newNode.rot = rotation_matrix;
  //this->xyz_node_list.push_back(newNode);
  this->xyz_node_list[0].rot = rotation_matrix;

  for (int i = 0; i < 3; i++){
    this->xyz_node_list[0].xyz[i] = this->tree[0].offset[i];
    int pind = this->tree[0].posind[i];
    if (pind >= 0)
      this->xyz_node_list[0].xyz[i] = this->xyz_node_list[0].xyz[i] + channels[pind % channels.size()][(int)(pind / channels.size())];
  }

  for (unsigned int i = 0; i < this->tree[0].children.size(); i++){
    int ind = this->tree[0].children[i];
    this->xyz_node_list = GetChildXyz(this->xyz_node_list, ind, channels);
  }

  std::vector< std::vector<float> > xyz(31, std::vector<float>(3));
  for (unsigned int i = 0; i < this->xyz_node_list.size(); i++){
    xyz[i][0] = this->xyz_node_list[i].xyz[0];
    xyz[i][1] = this->xyz_node_list[i].xyz[1];
    xyz[i][2] = this->xyz_node_list[i].xyz[2];

  }
  return xyz;
}
