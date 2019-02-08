/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "urdf/model.h"

static const std::string good_robot{"<robot name=\"myrobot\">"
                                    "  <link name=\"dummy\">"
                                    "  </link>"
                                    "</robot>"};

TEST(model_parser_initxml, initstring_bad_xml)
{
  const std::string robot{""};

  urdf::Model model;
  ASSERT_FALSE(model.initString(robot));
}

TEST(model_parser_initxml, initstring_good)
{
  urdf::Model model;
  ASSERT_TRUE(model.initString(good_robot));
}

TEST(model_parser_initxml, initxml_tinyxml_element_bad)
{
  urdf::Model model;
  ASSERT_FALSE(model.initXml(reinterpret_cast<TiXmlElement *>(NULL)));
}

TEST(model_parser_initxml, initxml_tinyxml_element_good)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(good_robot.c_str());

  urdf::Model model;
  ASSERT_TRUE(model.initXml(xml_doc.RootElement()));
}

TEST(model_parser_initxml, initxml_tinyxml_document_bad)
{
  urdf::Model model;
  ASSERT_FALSE(model.initXml(reinterpret_cast<TiXmlDocument *>(NULL)));
}

TEST(model_parser_initxml, initxml_tinyxml_document_good)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(good_robot.c_str());

  urdf::Model model;
  ASSERT_TRUE(model.initXml(&xml_doc));
}

TEST(model_parser_initxml, initxml_tinyxml2_element_bad)
{
  urdf::Model model;
  ASSERT_FALSE(model.initXml(reinterpret_cast<tinyxml2::XMLElement *>(NULL)));
}

TEST(model_parser_initxml, initxml_tinyxml2_element_good)
{
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(good_robot.c_str());

  urdf::Model model;
  ASSERT_TRUE(model.initXml(xml_doc.RootElement()));
}

TEST(model_parser_initxml, initxml_tinyxml2_document_bad)
{
  urdf::Model model;
  ASSERT_FALSE(model.initXml(reinterpret_cast<tinyxml2::XMLDocument *>(NULL)));
}

TEST(model_parser_initxml, initxml_tinyxml2_document_good)
{
  tinyxml2::XMLDocument xml_doc;
  xml_doc.Parse(good_robot.c_str());

  urdf::Model model;
  ASSERT_TRUE(model.initXml(&xml_doc));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
