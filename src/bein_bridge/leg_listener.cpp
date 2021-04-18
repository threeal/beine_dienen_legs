// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <bein_bridge/leg_listener.hpp>

#include <memory>
#include <string>

namespace bein_bridge
{

LegListener::LegListener(rclcpp::Node::SharedPtr node, int listen_port)
: node(node),
  listener(std::make_shared<housou::StringListener>(listen_port)),
  x_position(0.0),
  y_position(0.0),
  z_position(0.0),
  x_orientation(0.0),
  y_orientation(0.0),
  z_orientation(0.0),
  a_adc_read(0.0),
  b_adc_read(0.0),
  c_adc_read(0.0),
  d_adc_read(0.0)
{
  // Initialize the position publisher
  {
    position_publisher = node->create_publisher<PositionMsg>(
      std::string(node->get_name()) + "/position", 10
    );

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Position publisher initialized on " << position_publisher->get_topic_name() << "!");
  }

  // Initialize the orientation publisher
  {
    orientation_publisher = node->create_publisher<OrientationMsg>(
      std::string(node->get_name()) + "/orientation", 10
    );

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Orientation publisher initialized on " << orientation_publisher->get_topic_name() << "!");
  }
}

LegListener::~LegListener()
{
  disconnect();
}

bool LegListener::connect()
{
  return listener->connect();
}

bool LegListener::disconnect()
{
  return listener->disconnect();
}

void LegListener::listen_process()
{
  auto message = listener->receive(64, ",");

  if (message.size() > 0) {
    try {
      size_t i = 0;

      // Get position data
      {
        x_position = stod(message[i++]);
        y_position = stod(message[i++]);
        z_position = stod(message[i++]);

        // Publish position data
        {
          PositionMsg msg;

          msg.x = x_position;
          msg.y = y_position;
          msg.z = z_position;

          position_publisher->publish(msg);
        }
      }

      // Get orientation data
      {
        x_orientation = stod(message[i++]);
        y_orientation = stod(message[i++]);
        z_orientation = stod(message[i++]);

        // Publish orientation data
        {
          OrientationMsg msg;

          msg.x = x_orientation;
          msg.y = y_orientation;
          msg.z = z_orientation;

          orientation_publisher->publish(msg);
        }
      }

      // Get ADC data
      {
        a_adc_read = stod(message[i++]);
        b_adc_read = stod(message[i++]);
        c_adc_read = stod(message[i++]);
        d_adc_read = stod(message[i++]);
      }
    } catch (const std::out_of_range & err) {
      RCLCPP_WARN_STREAM(node->get_logger(), "Not all values are received! " << err.what());
    }
  }
}

}  // namespace bein_bridge
