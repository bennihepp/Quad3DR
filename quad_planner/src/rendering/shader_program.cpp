//==================================================
// shader_program.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 19, 2016
//==================================================

#include <quad_planner/rendering/shader_program.h>
#include <stdexcept>
#include <vector>
#include <fstream>
#include <GL/glew.h>

using namespace quad_planner::rendering;


ShaderProgram::ShaderProgram()
: program_id_(0)
{
}

ShaderProgram::~ShaderProgram()
{
  clear();
}

void ShaderProgram::clear()
{
  if (program_id_ != 0)
  {
    for (GLuint shader_id : shader_ids_)
    {
      glDetachShader(program_id_, shader_id);
      glDeleteShader(shader_id);
    }
    glDeleteProgram(program_id_);
    program_id_ = 0;
    shader_ids_.clear();
  }
}

void ShaderProgram::init()
{
  clear();
  program_id_ = glCreateProgram();
  if (program_id_ == 0)
  {
    throw std::runtime_error("ShaderProgram::init() Failed to create shader program");
  }
}

void ShaderProgram::ensureInitialized()
{
  if (program_id_ == 0)
  {
    init();
  }
}

void ShaderProgram::compileShader(const std::string &source, ShaderType type)
{
  ensureInitialized();
  GLuint shader_id = glCreateShader(type);
  const char *source_pointer = source.c_str();
  glShaderSource(shader_id, 1, &source_pointer, nullptr);
  glCompileShader(shader_id);

  // Check shader compilation
  GLint result;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (result == GL_FALSE)
  {
    GLint log_length = 0;
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_length);
    std::vector<char> shader_error_message(log_length+1);
    glGetShaderInfoLog(shader_id, log_length, nullptr, &shader_error_message[0]);
    std::cerr << "glCompileShader() failed [type=" << type << "]: " << &shader_error_message[0] << std::endl;
    std::cerr << "Shader code: " << source << std::endl;
    throw std::runtime_error("ShaderProgram::compileShader() failed: Unable to compile shader of type " + getShaderTypeString(type));
  }
  else
  {
    glAttachShader(program_id_, shader_id);
    shader_ids_.push_back(shader_id);
  }
}

void ShaderProgram::compileShaderFromStream(std::istream &input, ShaderType type)
{
  if (input.good())
  {
    std::string source;
    std::string line = "";
    while (std::getline(input, line))
    {
      source += "\n" + line;
    }
    compileShader(source, type);
  }
  else
  {
    throw std::runtime_error("ShaderProgram::compileShaderFromStream() failed: Invalid input stream for shader code");
  }
}

void ShaderProgram::compileShaderFromFile(const std::string &filename, ShaderType type)
{
  std::ifstream input(filename, std::ios::in);
  if (input.is_open())
  {
    try {
      compileShaderFromStream(input, type);
    }
    catch (const std::runtime_error& err) {
      throw std::runtime_error(std::string("Error when trying to compile shader from file '") + filename + "': " + err.what());
    }
    input.close();
  }
  else
  {
    throw std::runtime_error(std::string("ShaderProgram::compileShaderFromFile() failed: Unable to open file for shader code: ") + filename);
  }
}

void ShaderProgram::link()
{
  ensureInitialized();
  glLinkProgram(program_id_);

  // Check program linkage
  GLint result;
  glGetProgramiv(program_id_, GL_LINK_STATUS, &result);
  if (result == GL_FALSE)
  {
    GLint log_length = 0;
    glGetProgramiv(program_id_, GL_INFO_LOG_LENGTH, &log_length);
    std::vector<char> program_error_message(log_length+1);
    glGetProgramInfoLog(program_id_, log_length, nullptr, &program_error_message[0]);
    std::cerr << "glLinkProgram() failed: " << &program_error_message[0] << std::endl;
    throw std::runtime_error("ShaderProgram::link() failed: Failed to link shader program");
  }

  // Validate program
  glValidateProgram(program_id_);
  glGetProgramiv(program_id_, GL_VALIDATE_STATUS, &result);

  if (result == GL_FALSE)
  {
    GLint log_length = 0;
    glGetProgramiv(program_id_, GL_INFO_LOG_LENGTH, &log_length);
    std::vector<char> program_error_message(log_length+1);
    glGetProgramInfoLog(program_id_, log_length, nullptr, &program_error_message[0]);
    std::cerr << "glValidateProgram() failed: " << &program_error_message[0] << std::endl;
    throw std::runtime_error("ShaderProgram::link() failed: Failed to validate shader program");

  }
}

void ShaderProgram::use()
{
  glUseProgram(program_id_);
}

bool ShaderProgram::hasUniformLocation(const std::string &name)
{
  GLint location;
  auto it = uniform_locations_.find(name);
  if (it == uniform_locations_.end())
  {
    GLint location = glGetUniformLocation(program_id_, name.c_str());
    uniform_locations_[name] = location;
  }
  else
  {
    location = it->second;
  }
  return location != -1;
}

GLint ShaderProgram::getUniformLocation(const std::string &name, bool fail_if_not_found /*= true*/)
{
  GLint location;
  auto it = uniform_locations_.find(name);
  if (it == uniform_locations_.end())
  {
    location = glGetUniformLocation(program_id_, name.c_str());
    uniform_locations_[name] = location;
  }
  else
  {
    location = it->second;
  }
  if (fail_if_not_found && location == -1)
  {
    std::cerr << "glGetUniformLocation() failed: name=" << name << std::endl;
    throw std::runtime_error("ShaderProgram::getUniformLocation() failed: Failed to get uniform location for shader program");
  }
  return location;
}

void ShaderProgram::setUniform(const std::string &name, float value)
{
  GLint location = getUniformLocation(name, false);
  if (location != -1) {
    glUniform1f(location, value);
  }
  else {
    std::cerr << "WARNING: Uniform '" << name << "' is not used by shader" << std::endl;
  }
}
void ShaderProgram::setUniform(const std::string &name, const glm::mat4 &matrix)
{
  GLint location = getUniformLocation(name, false);
  if (location != -1) {
    glUniformMatrix4fv(location, 1, GL_FALSE, &matrix[0][0]);
  }
//  else {
//    std::cerr << "WARNING: Uniform '" << name << "' is not used by shader" << std::endl;
//  }
}

void ShaderProgram::setUniform(const std::string &name, const Eigen::MatrixXd &matrix)
{
  Eigen::MatrixXf matrix_float(matrix.rows(), matrix.cols());
  for (int col = 0; col < matrix.cols(); ++col)
  {
    for (int row = 0; row < matrix.rows(); ++row)
    {
      matrix_float(row, col) = static_cast<float>(matrix(row, col));
    }
  }
  setUniform(name, matrix_float);
}

void ShaderProgram::setUniform(const std::string &name, const Eigen::MatrixXf &matrix)
{
  assert(matrix.rows() == 4);
  assert(matrix.cols() == 4);
  GLint location = getUniformLocation(name, false);
  if (location != -1) {
    glUniformMatrix4fv(location, 1, GL_FALSE, matrix.data());
  }
//  else {
//    std::cerr << "WARNING: Uniform '" << name << "' is not used by shader" << std::endl;
//  }
}
