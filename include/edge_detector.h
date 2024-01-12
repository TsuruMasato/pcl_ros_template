#pragma once
#include <ros/ros.h>

namespace edge_detector
{

class EdgeDetector {
public:
  EdgeDetector();
  ~EdgeDetector();

  void detectEdges();
};

} // namespace edge_detector