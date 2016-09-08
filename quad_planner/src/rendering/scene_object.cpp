//==================================================
// scene_object.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 22, 2016
//==================================================

#include <quad_planner/rendering/scene_object.h>

namespace quad_planner
{
namespace rendering
{

SceneObject::SceneObject()
: obj_trans_(1.0f), visible_(false)
{
}

SceneObject::SceneObject(const std::shared_ptr<RenderObject> &obj_ptr, const std::shared_ptr<ShaderProgram> &shader_ptr)
: obj_ptr_(obj_ptr), shader_ptr_(shader_ptr), obj_trans_(1.0f), visible_(true)
{
}

SceneObject::~SceneObject()
{
}


std::shared_ptr<RenderObject> SceneObject::getRenderObject()
{
  return obj_ptr_;
}

std::shared_ptr<const RenderObject> SceneObject::getRenderObject() const
{
  return obj_ptr_;
}

void SceneObject::setRenderObject(const std::shared_ptr<RenderObject> &obj_ptr)
{
  obj_ptr_ = obj_ptr;
}

void SceneObject::setShaderProgram(const std::shared_ptr<ShaderProgram> &shader_ptr)
{
  shader_ptr_ = shader_ptr;
}

bool SceneObject::isVisible() const{
  return visible_;
}

void SceneObject::setVisible(bool visible)
{
  visible_ = visible;
}


const glm::mat4& SceneObject::getTransformation() const
{
  return obj_trans_;
}

glm::mat4& SceneObject::getTransformation()
{
  return obj_trans_;
}

void SceneObject::setTransformation(const glm::mat4 &trans)
{
  obj_trans_ = trans;
}


void SceneObject::render(const Eigen::MatrixXd &mvp) const
{
  render(Utilities::eigenToGlm(mvp));
}

void SceneObject::render(const Eigen::MatrixXf &mvp) const
{
  render(Utilities::eigenToGlm(mvp));
}

void SceneObject::render(const Eigen::Matrix<double, 4, 4> &mvp) const
{
  render(Utilities::eigenToGlm(mvp));
}

void SceneObject::render(const Eigen::Matrix<float, 4, 4> &mvp) const
{
  render(Utilities::eigenToGlm(mvp));
}

void SceneObject::render(const glm::mat4 &mvp) const
{
  if (isVisible())
  {
    if (!obj_ptr_ || !shader_ptr_)
    {
      throw std::runtime_error("SceneObject::render() failed: Not initialized properly.");
    }
    shader_ptr_->use();
    glm::mat4 obj_mvp = mvp * obj_trans_;
    shader_ptr_->setUniform("mvp", obj_mvp);
    obj_ptr_->render();
  }
}

} /* namespace rendering */
} /* namespace quad_planner */
