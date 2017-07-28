/*
 * main.cpp
 * Created: July 27, 2017
 * Author: Stephen Mylabathula
 */

#include "MotionModel.h"
using namespace std;

int main() {
  MotionModel mm;
  mm.load("jump.txt");
  mm.print();
  return 0;
}
