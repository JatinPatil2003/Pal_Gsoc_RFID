/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <math.h>

#include <gz/msgs/stringmsg.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "DummySensor.hh"

using namespace custom;


bool DummySensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("dummysensor" != type)
  {
    gzerr << "Trying to load [dummysensor] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<gz::msgs::StringMsg>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:dummysensor"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }
  gzdbg << "DummySensor Loaded!!" << std::endl;
  return true;
}

bool DummySensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool DummySensor::Update(const std::chrono::steady_clock::duration &_now)
{
  gz::msgs::StringMsg msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  msg.set_data("Hello World!");

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}
