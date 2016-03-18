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

#ifndef URDF_SENSOR_H
#define URDF_SENSOR_H

#include <string>
#include <map>
#include <urdf_parser/sensor_parser.h>

namespace urdf {

/** Retrieve sensor parsers available through the plugin-lib mechanism
    whose name matches any of the names listed in allowed.
    If allowed is empty (the default), all parsers will be returned.
*/
urdf::SensorParserMap getSensorParsers(const std::vector<std::string> &allowed = std::vector<std::string>());

/** Conveniency method returning the SensorParserMap for the given sensor name */
urdf::SensorParserMap getSensorParser(const std::string &name);

/** parse <sensor> tags in URDF document */
SensorMap parseSensors(const std::string &xml, const urdf::SensorParserMap &parsers);
SensorMap parseSensorsFromParam(const std::string &param, const urdf::SensorParserMap &parsers);
SensorMap parseSensorsFromFile(const std::string &filename, const urdf::SensorParserMap &parsers);

}

#endif
