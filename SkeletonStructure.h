/*
 * SkeletonStructure.h
 * Created: July 27, 2017
 * Author: Stephen Mylabathula
 */

#ifndef SKELETONSTRUCTURE_H_
#define SKELETONSTRUCTURE_H_
#include <string>
#include <vector>
#include <Eigen/Dense>

struct JointNode {
  std::string name;
  int id;
  float offset[3];
  std::vector<float> orientation;
  float axis[3];
  std::string axis_order;
  Eigen::Matrix3f c;
  Eigen::Matrix3f c_inv;
  std::vector<std::string> channels;
  float bodymass;
  float confmass;
  int parent;
  std::string order;
  int rotind[3];
  int posind[3];
  std::vector<int> children;
  std::vector< std::vector<float> > limits;
};

struct XYZNode {
  Eigen::Matrix3f rot;
  float xyz[3];
};

class SkeletonStructure {
 private:
  std::vector<XYZNode> xyz_node_list;
  void GetChildXyz(int ind, std::vector< std::vector<float> > channels);
  Eigen::Matrix3f RotationMatrix(float xangle, float yangle, float zangle, std::string order);
 public:
  int length;
  int mass;
  std::string angle;
  std::string type;
  std::string documentation;
  std::string name;
  std::vector<JointNode> tree;
  float model_height;
  int head_index;
  int left_foot_index;
  int right_foot_index;

  // constructors
  SkeletonStructure();

  // destructor
  ~SkeletonStructure();

  // method to compute skeleton xyz
  std::vector< std::vector<float> > ComputeSkeletonXYZPose(std::vector< std::vector<float> > channels);
};


#endif  // SKELETONSTRUCTURE_H_
