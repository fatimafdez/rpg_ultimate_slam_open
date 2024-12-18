// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#pragma once

#include <memory>
#include <ze/visualization/viz_interface.hpp>

// fwd
namespace ros {
class Publisher;
class NodeHandle;
}

namespace tf {
class TransformBroadcaster;
}

namespace ze {

//! @todo(cfo): Deal with multi-threaded publishing.
class VisualizerRos : public Visualizer
{
public:

  VisualizerRos();
  VisualizerRos(const std::string& frame);

  virtual ~VisualizerRos() = default;

  // ---------------------------------------------------------------------------
  // Draw single elements

  virtual void drawPoint(
      const std::string& ns,
      const size_t id,
      const Position& point,
      const Color& color,
      const real_t size = 0.02) override;

  virtual void drawLine(
      const std::string& ns,
      const size_t id,
      const Position& line_from,
      const Position& line_to,
      const Color& color,
      const real_t size = 0.02) override;

  virtual void drawCoordinateFrame(
      const std::string& ns,
      const size_t id,
      const Transformation& pose, // T_W_B
      const real_t size = 0.2) override;

  virtual void drawRobot(
      const std::string& name,
      const Transformation& T_W_B,
      const ros::Time& stamp) override;

  // ---------------------------------------------------------------------------
  // Draw multiple elements

  virtual void drawPoints(
      const std::string& ns,
      const size_t id,
      const Positions& points,
      const Color& color,
      const real_t size = 0.02) override;

  virtual void drawLines(
      const std::string& ns,
      const size_t id,
      const LineMarkers& lines,
      const Color& color,
      const real_t size = 0.02) override;

  virtual void drawCoordinateFrames(
      const std::string& ns,
      const size_t id,
      const TransformationVector& poses,
      const real_t size = 0.2) override;

  virtual void drawTrajectory(
      const std::string& topic,
      const size_t id,
      const std::vector<Position>& points,
      const Color& color,
      const real_t size = 0.02) override;

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::Publisher> pub_marker_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  std::string world_frame = "odom";    //!< World-frame
  double viz_scale_ = 1.0;            //!< Scale marker size
};


} // namespace ze
