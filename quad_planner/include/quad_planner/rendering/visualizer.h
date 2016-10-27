//==================================================
// visualizer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <quad_planner/rendering/octomap_renderer.h>
#include <quad_planner/rendering/shader_program.h>
#include <quad_planner/rendering/scene_object.h>
#include <quad_planner/quad_planner.h>

namespace quad_planner
{
namespace rendering
{

class Visualizer
{
  GLFWwindow *window_;
  int width_;
  int height_;

  // Input states
  bool left_button_pressed_;
  bool right_button_pressed_;

  std::shared_ptr<ShaderProgram> octomap_shader_ptr_;
  std::shared_ptr<ShaderProgram> arcball_shader_ptr_;
  std::shared_ptr<ShaderProgram> trajectory_shader_ptr_;
  SceneObject octomap_so_;
  SceneObject so_;
  SceneObject so2_;
  SceneObject arcball1_so_;
  SceneObject arcball2_so_;

  GLuint vertex_array_id_;

  glm::mat4 view_;
  glm::mat4 projection_;
  const float ARCBALL_ROTATION_THRESHOLD = 1e-6f;
  const float DEFAULT_MODEL_SCALE = 0.1f;

  // Shaders
  static const char *SIMPLE_VERTEX_SHADER;
  static const char *ELEVATION_VERTEX_SHADER;
  static const char *SIMPLE_FRAGMENT_SHADER;

public:
  Visualizer();
  virtual ~Visualizer();

  void init();
  void init(int core_minor, int core_major);

  std::shared_ptr<OctomapRenderer> getOctomapRenderer();

  // GLFW callbacks
//  static void glfwWindowSizeCallback(GLFWwindow *window, int width, int height);
  static void glfwFramebufferSizeCallbackStatic(GLFWwindow *window, int width, int height);
  static void glfwKeyCallbackStatic(GLFWwindow *window, int key, int scancode, int action, int mods);
  static void glfwScrollCallbackStatic(GLFWwindow *window, double xoffset, double yoffset);
  static void glfwCursorPosCallbackStatic(GLFWwindow *window, double xpos, double ypos);
  static void glfwMouseButtonCallbackStatic(GLFWwindow *window, int button, int action, int mods);
  void glfwFramebufferSizeCallback(GLFWwindow *window, int width, int height);
  void glfwKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
  void glfwScrollCallback(GLFWwindow *window, double xoffset, double yoffset);
  void glfwCursorPosCallback(GLFWwindow *window, double xpos, double ypos);
  void glfwMouseButtonCallback(GLFWwindow *window, int button, int action, int mods);

  GLuint loadShader(const std::string &vertex_shader_filename, const std::string &fragment_shader_filename);

  glm::vec3 getArcballVector(int cursor_pos_x, int cursor_pos_y) const;
  void resetView(bool reset_scale = true);
  void updateMatricesFromInputs();

  void run(std::shared_ptr<ob::ProblemDefinition>);
  void render();
};

}
}
