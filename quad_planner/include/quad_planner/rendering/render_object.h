//==================================================
// RenderObject.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 23, 2016
//==================================================

#pragma once

namespace quad_planner
{
namespace rendering
{

class RenderObject
{
public:
  RenderObject()
  {
  }

  virtual ~RenderObject()
  {
  }

  virtual void render() const = 0;
};

} /* namespace rendering */
} /* namespace quad_planner */
