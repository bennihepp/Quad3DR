//==================================================
// scene_object.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 22, 2016
//==================================================

#pragma once

#include <quad_planner/rendering/linalg.h>
#include <memory>
#include <quad_planner/rendering/shader_program.h>
#include <quad_planner/rendering/render_object.h>

namespace quad_planner
{
namespace rendering
{

class SceneObject
{
  std::shared_ptr<RenderObject> obj_ptr_;
  std::shared_ptr<ShaderProgram> shader_ptr_;
  glm::mat4 obj_trans_;
  bool visible_;

public:
  explicit SceneObject();
  explicit SceneObject(const std::shared_ptr<RenderObject> &obj_ptr, const std::shared_ptr<ShaderProgram> &shader_ptr);
  virtual ~SceneObject();

  std::shared_ptr<RenderObject> getRenderObject();
  std::shared_ptr<const RenderObject> getRenderObject() const;
  void setRenderObject(const std::shared_ptr<RenderObject> &obj_ptr);
  void setShaderProgram(const std::shared_ptr<ShaderProgram> &shader_ptr);
  bool isVisible() const;
  void setVisible(bool visible=true);

  const glm::mat4& getTransformation() const;
  glm::mat4& getTransformation();
  void setTransformation(const glm::mat4 &trans);

  void render(const Eigen::MatrixXd &mvp) const;
  void render(const Eigen::Matrix<double, 4, 4> &mvp) const;
  void render(const Eigen::MatrixXf &mvp) const;
  void render(const Eigen::Matrix<float, 4, 4> &mvp) const;
  void render(const glm::mat4 &mvp) const;
};

} /* namespace rendering */
} /* namespace quad_planner */
