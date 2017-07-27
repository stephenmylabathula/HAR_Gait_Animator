/*
 * MotionModel.h
 * Created: July 26, 2017
 * Author: Stephen Mylabathula
 */

//TODO: change any vectors to arrays if possible

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_
#include <string>
#include <vector>

struct JointNode {
  std::string name;
  int id;
  float offset[3];
  std::vector<float> orientation;
  float axis[3];
  std::string axis_order;
  //TODO: add eigen matrix for C and Cinv
  std::vector<std::string> channels;
  float bodymass;
  float confmass;
  int parent;
  std::string order;
  int rotind[3];
  int posind[3];
  std::vector<int> children;
  std::vector<float> limits;
};

struct SkeletonStructure {
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
};

struct MotionPhases {
  std::vector<std::string> titles;
  std::vector<int> indices;
  int start_index;
  int end_index;
};

class MotionModel {
 private:
  float frame_length;
  SkeletonStructure skeleton_structure;
  std::vector<std::vector<float>> channels;
  MotionPhases motion_phases;
  std::string name;

 public:
  // constructors
  MotionModel();

  // destructor
  ~MotionModel();

  // load data from CSV
  void load(std::string filePath);
};

#endif  // MOTIONMODEL_H_
