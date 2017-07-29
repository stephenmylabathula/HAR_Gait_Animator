/*
 * MotionModel.h
 * Created: July 26, 2017
 * Author: Stephen Mylabathula
 */

//TODO: change any vectors to arrays if possible

/*
 * NOTE - The following fields have been zeroed by subtracting one from their value:
 * Skeleton - Head Index
 *          - Left Foot index
 *          - Right Foot Index
 * Tree - Children
 *      - PosInd
 *      - RotInd
 * Phase - Start Index
 *       - End index
 *       - Indices
 */

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#include <string>
#include <vector>
#include "SkeletonStructure.h"

struct MotionPhases {
  std::vector<std::string> titles;
  std::vector<int> indices;
  int start_index;
  int end_index;
};

class MotionModel {
 private:
  float frame_length;
  std::vector< std::vector<float> > channels;
  SkeletonStructure skeleton_structure;
  MotionPhases motion_phases;
  std::string name;

 public:
  // constructor
  MotionModel();

  // destructor
  ~MotionModel();

  // load data from CSV
  void load(std::string file_path);

  // print key content for sanity check
  void print(void);
};

#endif  // MOTIONMODEL_H_
