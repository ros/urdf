/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Wim Meeussen */

#ifndef URDF__MODEL_H_
#define URDF__MODEL_H_

#include <string>

#include <urdf_model/model.h>

#include <urdf/urdfdom_compatibility.h>

#include <tinyxml.h>

#include <tinyxml2.h>

#include <ros/ros.h>

#include "urdf/visibility_control.hpp"

namespace urdf
{

class Model : public ModelInterface
{
public:
  /// \brief Load Model from TiXMLElement
  URDF_EXPORT URDF_DEPRECATED("TinyXML API is deprecated, use the TinyXML2 version instead") bool initXml(TiXmlElement * xml);
  /// \brief Load Model from TiXMLDocument
  URDF_EXPORT URDF_DEPRECATED("TinyXML API is deprecated, use the TinyXML2 version instead") bool initXml(TiXmlDocument * xml);
  /// \brief Load Model from tinyxml2::XMLElement
  URDF_EXPORT bool initXml(const tinyxml2::XMLElement *xml);
  /// \brief Load Model from tinyxml2::XMLDocument
  URDF_EXPORT bool initXml(const tinyxml2::XMLDocument *xml);
  /// \brief Load Model given a filename
  URDF_EXPORT bool initFile(const std::string & filename);
  /// \brief Load Model given the name of a parameter on the parameter server
  URDF_EXPORT bool initParam(const std::string & param);
  /// \brief Load Model given the name of parameter on parameter server using provided nodehandle
  URDF_EXPORT bool initParamWithNodeHandle(const std::string & param,
    const ros::NodeHandle & nh = ros::NodeHandle());
  /// \brief Load Model from a XML-string
  URDF_EXPORT bool initString(const std::string & xmlstring);
};

// shared_ptr declarations moved to urdf/urdfdom_compatibility.h to allow for
// std::shared_ptrs in latest version

}  // namespace urdf

#endif  // URDF__MODEL_H_
