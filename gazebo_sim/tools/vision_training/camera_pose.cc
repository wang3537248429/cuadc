// Persistent Gazebo transport client for offline RGB dataset capture.
#include <iostream>
#include <string>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

int main() {
  gz::transport::Node node;
  double x, y, z, qx, qy, qz, qw;
  while (std::cin >> x >> y >> z >> qx >> qy >> qz >> qw) {
    gz::msgs::Pose request;
    request.set_name("capture_camera");
    request.mutable_position()->set_x(x);
    request.mutable_position()->set_y(y);
    request.mutable_position()->set_z(z);
    request.mutable_orientation()->set_x(qx);
    request.mutable_orientation()->set_y(qy);
    request.mutable_orientation()->set_z(qz);
    request.mutable_orientation()->set_w(qw);
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = node.Request("/world/dataset/set_pose", request, 3000, response, result);
    std::cout << (executed && result && response.data() ? "ok" : "failed") << std::endl;
  }
}
