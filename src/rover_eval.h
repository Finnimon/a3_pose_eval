#pragma once
#include "./pose_eval.h"
#include <array>
#include <deque>

namespace a3_math {
typedef std::array<vec2, 2> tag_positions;

void print_pose(const pose& pose){
    std::cout<<"pos "<<pose.pos.x<<"  "<<pose.pos.y<<" front "<<pose.front.x<<"  "<<pose.front.y<<"\n";
}
pose find_rover_pose(const pose &initial_rob_pose, const tag_positions &initial_tags,
               const tag_positions &measured) {
  const pose frame_initial = {
      lerp(initial_tags[0], initial_tags[1], 0.5f),
      normalize(sub(initial_tags[1],initial_tags[0]))
  };
  print_pose(frame_initial);
  const pose frame_meas = {
      lerp(measured[0], measured[1], 0.5f),
      normalize(sub(measured[1],measured[0]))
  };
  print_pose(frame_meas);
  return to_world(frame_meas,to_local(frame_initial, initial_rob_pose));
}
} // namespace a3_pose
