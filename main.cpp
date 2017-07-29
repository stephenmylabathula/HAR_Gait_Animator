/*
 * main.cpp
 * Created: July 27, 2017
 * Author: Stephen Mylabathula
 */

#include "MotionModel.h"
using namespace std;

int main() {
  // Create a Motionmodel object
  MotionModel mm;
  // Load in the jump motion model
  mm.load("jump.txt");
  // Print info about the loaded model
  mm.print();
  return 0;
}
