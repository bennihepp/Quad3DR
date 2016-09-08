//==================================================
// shader_program.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 19, 2016
//==================================================

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <GL/glew.h>
#include <quad_planner/rendering/linalg.h>

namespace quad_planner
{
namespace rendering
{

class ShaderProgram
{
  GLuint program_id_;
  std::vector<GLuint> shader_ids_;
  std::map<std::string, GLint> uniform_locations_;

  void init();
  void clear();
  void ensureInitialized();

public:
  enum ShaderType
  {
    VERTEX = GL_VERTEX_SHADER,
    FRAGMENT = GL_FRAGMENT_SHADER,
    GEOMETRY = GL_GEOMETRY_SHADER,
  };

  ShaderProgram();
  virtual ~ShaderProgram();

  void compileShader(const std::string &source, ShaderType type);
  void compileShaderFromStream(std::istream &input, ShaderType type);
  void compileShaderFromFile(const std::string &filename, ShaderType type);
  void link();

  void use();
  GLint getUniformLocation(const std::string &name);
  void setUniform(const std::string &name, const glm::mat4 &matrix);
  void setUniform(const std::string &name, const Eigen::MatrixXd &matrix);
  void setUniform(const std::string &name, const Eigen::MatrixXf &matrix);
};

}
}
