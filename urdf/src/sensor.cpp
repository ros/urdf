/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2016 CITEC, Bielefeld University
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the authors nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Robert Haschke */

#include "urdf/sensor.h"

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>
#include <fstream>

namespace urdf {

SensorMap parseSensorsFromFile(const std::string &filename, const SensorParserMap &parsers)
{
  SensorMap result;
  std::ifstream stream(filename.c_str());
  if (!stream.is_open())
  {
     throw std::runtime_error("Could not open file [" + filename + "] for parsing.");
  }

  std::string xml_string((std::istreambuf_iterator<char>(stream)),
                         std::istreambuf_iterator<char>());
  return parseSensors(xml_string, parsers);
}


SensorMap parseSensorsFromParam(const std::string &param, const SensorParserMap &parsers)
{
  ros::NodeHandle nh;
  std::string xml_string;
  
  // gets the location of the robot description on the parameter server
  std::string full_param;
  if (!nh.searchParam(param, full_param)){
    throw std::runtime_error("Could not find parameter " + param + " on parameter server");
  }

  // read the robot description from the parameter server
  if (!nh.getParam(full_param, xml_string)){
    throw std::runtime_error("Could not read parameter " + param + " on parameter server");
  }
  return parseSensors(xml_string, parsers);
}


SensorMap parseSensors(const std::string &xml_string, const SensorParserMap &parsers)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
     throw std::runtime_error(std::string("Could not parse the xml document: ") + xml_doc.ErrorDesc());
  return parseSensors(xml_doc, parsers);
}

const SensorParserMap& getDefaultSensorParserMap()
{
  static boost::mutex PARSER_PLUGIN_LOCK;
  static boost::shared_ptr<pluginlib::ClassLoader<urdf::SensorParser> > PARSER_PLUGIN_LOADER;
  static SensorParserMap defaultParserMap;

  boost::mutex::scoped_lock _(PARSER_PLUGIN_LOCK);
  try
  {
    if (!PARSER_PLUGIN_LOADER) {
      PARSER_PLUGIN_LOADER.reset(new pluginlib::ClassLoader<urdf::SensorParser>("urdf", "urdf::SensorParser"));

      const std::vector<std::string> &classes = PARSER_PLUGIN_LOADER->getDeclaredClasses();
      for (std::size_t i = 0 ; i < classes.size() ; ++i)
      {
         urdf::SensorParserSharedPtr parser;
         try {
            parser = PARSER_PLUGIN_LOADER->createInstance(classes[i]);
         } catch(const pluginlib::PluginlibException& ex) {
           ROS_ERROR_STREAM("Failed to create sensor parser: " << classes[i] << "\n" << ex.what());
         }
         defaultParserMap.insert(std::make_pair(classes[i], parser));
         ROS_DEBUG_STREAM("added sensor parser: " << classes[i]);
      }
      if (defaultParserMap.empty())
        ROS_WARN_STREAM("No sensor parsers found");
    }
  }
  catch(const pluginlib::PluginlibException& ex)
  {
     ROS_ERROR_STREAM("Exception while creating sensor plugin loader " << ex.what());
  }
  return defaultParserMap;
}

} // namespace
