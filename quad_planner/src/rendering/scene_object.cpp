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
: model_trans_(1.0f), visible_(false)
{
}

SceneObject::SceneObject(const std::shared_ptr<RenderObject> &obj_ptr, const std::shared_ptr<ShaderProgram> &shader_ptr)
: obj_ptr_(obj_ptr), shader_ptr_(shader_ptr), model_trans_(1.0f), visible_(true)
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


void SceneObject::updateViewportSize(float width, float height)
{
  shader_ptr_->use();
  shader_ptr_->setUniform("viewportWidth", width);
  shader_ptr_->setUniform("viewportWidth", height);
}

const glm::mat4& SceneObject::getTransformation() const
{
  return model_trans_;
}

glm::mat4& SceneObject::getTransformation()
{
  return model_trans_;
}

void SceneObject::setTransformation(const glm::mat4 &trans)
{
  model_trans_ = trans;
}


void SceneObject::render(const Eigen::MatrixXd& view, const Eigen::MatrixXd &projection, float width, float height) const
{
  render(Utilities::eigenToGlm(view), Utilities::eigenToGlm(projection), width, height);
}

void SceneObject::render(const Eigen::MatrixXf& view, const Eigen::MatrixXf& projection, float width, float height) const
{
  render(Utilities::eigenToGlm(view), Utilities::eigenToGlm(projection), width, height);
}

void SceneObject::render(const glm::mat4 &view, const glm::mat4 &projection, float width, float height) const
{
  if (isVisible())
  {
    if (!obj_ptr_ || !shader_ptr_)
    {
      throw std::runtime_error("SceneObject::render() failed: Not initialized properly.");
    }
    shader_ptr_->use();
    shader_ptr_->setUniform("viewportWidth", width);
    shader_ptr_->setUniform("viewportHeight", height);
    glm::mat4 mvp = projection * view * model_trans_;
    shader_ptr_->setUniform("MVP", mvp);
    shader_ptr_->setUniform("M", model_trans_);
    shader_ptr_->setUniform("V", view);
    obj_ptr_->render();
  }
}

} /* namespace rendering */
} /* namespace quad_planner */
