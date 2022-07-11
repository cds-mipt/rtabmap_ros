/*
 * Based on rviz/default_plugin/map_display.cpp
 * 
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ColoredOccupancyGrid.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/grid.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>
#include <rviz/display_context.h>

#include <Eigen/Dense>

namespace rtabmap_ros
{

ColoredOccupancyGridDisplay::ColoredOccupancyGridDisplay()
  : loaded_(false)
  , resolution_(0.0f)
  , size_(0)
{
  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
                                            "Amount of transparency to apply to the map.",
                                            this, SLOT( updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  draw_under_property_ = new rviz::BoolProperty("Draw Behind", false,
                                                "Rendering option, controls whether or not the map is always"
                                                " drawn behind everything else.",
                                                this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz::FloatProperty("Resolution", 0,
                                                 "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);
  
  position_property_ = new rviz::VectorProperty("Position", Ogre::Vector3::ZERO,
                                                "Position of the bottom left corner of the map, in meters. (not editable)",
                                                this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz::QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                                                       "Orientation of the map. (not editable)",
                                                       this);
  orientation_property_->setReadOnly(true);

  occ_property_ = new rviz::FloatProperty("Occupancy threshold", 0.6,
                                          "Occupancy amount at which object is considered dynamic.",
                                          this);
  occ_property_->setMin(0.0f);
  occ_property_->setMax(1.0f);

  mahalanobis_property_ = new rviz::FloatProperty("Mahalanobis distance", 6.0,
                                                  "Mahalanobis distance at which object is considered dynamic.",
                                                  this);
  mahalanobis_property_->setMin(0.0f);
}

ColoredOccupancyGridDisplay::~ColoredOccupancyGridDisplay()
{
  clear();
}

void ColoredOccupancyGridDisplay::onInitialize()
{
  MFDClass::onInitialize();

  static int count = 0;
  std::stringstream ss;
  ss << "DOGMObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  updateAlpha();
}

void ColoredOccupancyGridDisplay::reset()
{
  MFDClass::reset();
  clear();
}

void ColoredOccupancyGridDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;

  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha);

  if (alpha < 0.9998)
  {
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setDepthWriteEnabled(false);
  }
  else
  {
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(!draw_under_property_->getValue().toBool());
  }
}

void ColoredOccupancyGridDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998)
  {
    material_->setDepthWriteEnabled(!draw_under);
  }

  if (manual_object_)
  {
    if (draw_under)
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    }
    else
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }
}

void ColoredOccupancyGridDisplay::clear()
{
  setStatus(rviz::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_)
  {
    return;
  }

  scene_manager_->destroyManualObject(manual_object_);
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload(tex_name);

  loaded_ = false;
}

void ColoredOccupancyGridDisplay::processMessage(const rtabmap_ros::ColoredOccupancyGrid::ConstPtr& msg)
{
  if (msg->data.empty())
  {
    return;
  }

  // if (msg->info.size == 0)
  // {
  //   std::stringstream ss;
  //   ss << "Map is zero-sized (" << msg->info.size << ")";
  //   setStatus(rviz::StatusProperty::Error, "DOGM", QString::fromStdString(ss.str()));
  //   return;
  // }

  clear();

  setStatus(rviz::StatusProperty::Ok, "Message", "Map received" );

  // ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
  //            msg->info.size,
  //            msg->info.size,
  //            msg->info.resolution);

  float resolution = msg->info.resolution;
  int width = msg->info.width;
  int height = msg->info.height;
  // int size = msg->info.size;

  Ogre::Vector3 position(msg->info.origin.position.x,
                          msg->info.origin.position.y,
                          msg->info.origin.position.z);
                          
  Ogre::Quaternion orientation(msg->info.origin.orientation.w,
                                msg->info.origin.orientation.x,
                                msg->info.origin.orientation.y,
                                msg->info.origin.orientation.z);

  frame_ = msg->header.frame_id;
  // if (frame_.empty())
  // {
  //   frame_ = "/dogm";
  // }

  // Expand it to be RGB data
  unsigned int pixels_size = width * height;
  unsigned char* pixels = new unsigned char[pixels_size * 4];
  memset(pixels, 255, pixels_size * 4);

  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  // if( pixels_size != msg->data.size() )
  // {
  //   std::stringstream ss;
  //   ss << "Data size doesn't match size * size: size = " << size
  //      << ", data size = " << msg->data.size();
  //   setStatus( rviz::StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
  //   map_status_set = true;

  //   // Keep going, but don't read past the end of the data.
  //   if(msg->data.size() < pixels_size)
  //   {
  //     num_pixels_to_copy = msg->data.size();
  //   }
  // }

  unsigned char* pixels_ptr = pixels;
  
  for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++)
  {
    char occ = msg->data[pixel_index];
    unsigned char r = msg->r[pixel_index];
    unsigned char g = msg->g[pixel_index];
    unsigned char b = msg->b[pixel_index];

    if (occ == -1)
    {
      *pixels_ptr++ = 128; // red
      *pixels_ptr++ = 128; // green
      *pixels_ptr++ = 128; // blue
      *pixels_ptr++ = 255; // alpha
    }
    else if (occ >= 50)
    {
      *pixels_ptr++ = r; // red
      *pixels_ptr++ = g; // green
      *pixels_ptr++ = b; // blue
      *pixels_ptr++ = 255; // alpha
    }
    else
    {
      *pixels_ptr++ = 255; // red
      *pixels_ptr++ = 255; // green
      *pixels_ptr++ = 255; // blue
      *pixels_ptr++ = 255; // alpha
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size * 4));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "DOGMTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                pixel_stream, width, height, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_2D,
                                                                0);

  if(!map_status_set)
  {
    setStatus( rviz::StatusProperty::Ok, "DOGM", "Map OK" );
  }

  delete [] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "DOGMObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject(ss2.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, resolution * height, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(resolution * width, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }
  manual_object_->end();

  if (draw_under_property_->getBool())
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  resolution_property_->setValue(resolution);
  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  latest_map_pose_ = msg->info.origin;
  transformMap();

  loaded_ = true;

  context_->queueRender();
}

void ColoredOccupancyGridDisplay::transformMap()
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), latest_map_pose_, position, orientation))
  {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
               qPrintable(getName()), frame_.c_str(), qPrintable(fixed_frame_));

    setStatus(rviz::StatusProperty::Error, "Transform",
               "No transform from [" + QString::fromStdString( frame_ ) + "] to [" + fixed_frame_ + "]");
  }
  else
  {
    setStatus(rviz::StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void ColoredOccupancyGridDisplay::fixedFrameChanged()
{
  transformMap();
}

}  // end namespace dogm_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ColoredOccupancyGridDisplay, rviz::Display)
