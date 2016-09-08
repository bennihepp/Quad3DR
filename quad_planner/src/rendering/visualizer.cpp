#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <quad_planner/rendering/visualizer.h>
#include <quad_planner/rendering/color.h>
#include <quad_planner/rendering/linalg.h>
#include <quad_planner/rendering/scene_object.h>
#include <quad_planner/rendering/triangle_mesh.h>

using namespace glm;
using namespace quad_planner::rendering;


static std::ostream& operator<<(std::ostream& stream, const glm::vec3 &vec)
{
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
  return stream;
}

static std::ostream& operator<<(std::ostream& stream, const glm::vec4 &vec)
{
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";
  return stream;
}

static std::ostream& operator<<(std::ostream& stream, const glm::mat4 &mat)
{
  stream << std::endl;
  stream << "(" << mat[0][0] << ", " << mat[1][0] << ", " << mat[2][0] << ", " << mat[3][0] << std::endl;
  stream << " " << mat[0][1] << ", " << mat[1][1] << ", " << mat[2][1] << ", " << mat[3][1] << std::endl;
  stream << " " << mat[0][2] << ", " << mat[1][2] << ", " << mat[2][2] << ", " << mat[3][2] << std::endl;
  stream << " " << mat[0][3] << ", " << mat[1][3] << ", " << mat[2][3] << ", " << mat[3][3] << ")" << std::endl;
  return stream;
}

Visualizer::Visualizer()
: window_(nullptr), vertex_array_id_(0),
  width_(0), height_(0),
  left_button_pressed_(false), right_button_pressed_(false)
{
}

Visualizer::~Visualizer()
{
}

void Visualizer::init()
{
  init(3, 3);
}

void Visualizer::init(int core_minor, int core_major)
{
  if (!glfwInit())
  {
    throw std::runtime_error("Unable to initialize GLFW");
  }

  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, core_major); // We want OpenGL 3.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, core_minor);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //We don't want the old OpenGL

  width_ = 1024;
  height_ = 768;
  // Open a window and create its OpenGL context
  window_ = glfwCreateWindow(width_, height_, "quad_planner::Visualizer", NULL, NULL);
  if (window_ == nullptr)
  {
    throw std::runtime_error("Failed to open GLFW window");
  }
  glfwMakeContextCurrent(window_); // Initialize GLEW
  glewExperimental = true; // Needed in core profile
  if (glewInit() != GLEW_OK)
  {
    throw std::runtime_error("Failed to initialize GLEW");
  }

  // Reduce tearing
  glfwSwapInterval(1);

  // Callback registration
  glfwSetWindowUserPointer(window_, static_cast<void*>(this));
  glfwSetFramebufferSizeCallback(window_, &Visualizer::glfwFramebufferSizeCallbackStatic);
  glfwSetKeyCallback(window_, &Visualizer::glfwKeyCallbackStatic);
  glfwSetScrollCallback(window_, &Visualizer::glfwScrollCallbackStatic);
  glfwSetCursorPosCallback(window_, &Visualizer::glfwCursorPosCallbackStatic);
  glfwSetMouseButtonCallback(window_, &Visualizer::glfwMouseButtonCallbackStatic);

  // Ensure we can capture keys being pressed below
  glfwSetInputMode(window_, GLFW_STICKY_KEYS, GL_TRUE);

  octomap_shader_ptr_ = std::make_shared<ShaderProgram>();
  octomap_shader_ptr_->compileShaderFromFile("shaders/simple_vertex_shader.vertexshader", ShaderProgram::VERTEX);
  octomap_shader_ptr_->compileShaderFromFile("shaders/simple_fragment_shader.fragmentshader", ShaderProgram::FRAGMENT);
  octomap_shader_ptr_->link();

  octomap_so_ = SceneObject(std::make_shared<OctomapRenderer>(), octomap_shader_ptr_);
  octomap_so_.setVisible(true);

  std::shared_ptr<ColorTriangleMesh> sphere_ptr = ColorTriangleMesh::createSphere(1.0, ColorEigen::red(), 3);
  so_ = SceneObject(sphere_ptr, octomap_shader_ptr_);
  so_.setTransformation(glm::translate(glm::vec3(0, 0, 1)));
  so_.setVisible(true);

  arcball1_so_.setShaderProgram(octomap_shader_ptr_);
  arcball1_so_.setRenderObject(sphere_ptr);

  arcball2_so_.setShaderProgram(octomap_shader_ptr_);
  arcball2_so_.setRenderObject(sphere_ptr);
}

std::shared_ptr<OctomapRenderer> Visualizer::getOctomapRenderer()
{
  return std::dynamic_pointer_cast<OctomapRenderer>(octomap_so_.getRenderObject());
}

void Visualizer::glfwFramebufferSizeCallbackStatic(GLFWwindow *window, int width, int height)
{
  static_cast<Visualizer*>(glfwGetWindowUserPointer(window))->glfwFramebufferSizeCallback(window, width, height);
}

void Visualizer::glfwKeyCallbackStatic(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  static_cast<Visualizer*>(glfwGetWindowUserPointer(window))->glfwKeyCallback(window, key, scancode, action, mods);
}

void Visualizer::glfwScrollCallbackStatic(GLFWwindow *window, double xoffset, double yoffset)
{
  static_cast<Visualizer*>(glfwGetWindowUserPointer(window))->glfwScrollCallback(window, xoffset, yoffset);
}

void Visualizer::glfwCursorPosCallbackStatic(GLFWwindow *window, double xpos, double ypos)
{
  static_cast<Visualizer*>(glfwGetWindowUserPointer(window))->glfwCursorPosCallback(window, xpos, ypos);
}

void Visualizer::glfwMouseButtonCallbackStatic(GLFWwindow *window, int button, int action, int mods)
{
  static_cast<Visualizer*>(glfwGetWindowUserPointer(window))->glfwMouseButtonCallback(window, button, action, mods);
}

void Visualizer::glfwFramebufferSizeCallback(GLFWwindow *window, int width, int height)
{
  glViewport(0, 0, width, height);
  width_ = width;
  height_ = height;
  resetView(false);
}

void Visualizer::glfwKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if (key == GLFW_KEY_Z && action == GLFW_PRESS)
  {
    resetView();
  }
  else if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
  {
    glfwSetWindowShouldClose(window_, true);
  }
}

void Visualizer::glfwScrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
//  double factor;
//  if (yoffset > 0)
//  {
//    factor = 1 / 0.9;
//  }
//  else if (yoffset < 0)
//  {
//    factor = 0.9;
//  }
//  model_ *= glm::scale(glm::vec3(factor, factor, factor));

  float model_scale = glm::length(model_[0]);
  float translate_speed = 0.1f / model_scale / model_scale;
  glm::mat4 inv_model = glm::transpose(model_);
  glm::vec4 translation_vec = inv_model * glm::vec4(0, 0, translate_speed * yoffset, 1);
  model_ *= glm::translate(glm::vec3(translation_vec));
}

void Visualizer::glfwCursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
  // TODO
}

void Visualizer::glfwMouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
  // TODO
}

glm::vec3 Visualizer::getArcballVector(int cursor_pos_x, int cursor_pos_y) const
{
  glm::vec3 point = glm::vec3(
    2.0 * cursor_pos_x / width_ - 1.0,
    2.0 * cursor_pos_y / height_ - 1.0,
    0
  );
  point.y = -point.y;
  float dist_squared = point.x * point.x + point.y * point.y;
  if (dist_squared <= 1.0f)
  {
    point.z = std::sqrt(1.0f - dist_squared);
  }
  else
  {
    std::cout << "arcball vector length: " << glm::length(point) << std::endl;
    std::cout << "out of reach" << std::endl;
    point = glm::normalize(point);
  }
  return point;
}

void Visualizer::resetView(bool reset_model)
{
  // Projection matrix: 110° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
  float aspect_ratio = 1.0f;
  if (width_ > 0)
  {
    aspect_ratio = static_cast<float>(width_) / height_;
  }
  projection_ = glm::perspective(glm::radians(45.0f), aspect_ratio, 0.1f, 100.0f);
  // Orthographic
  //glm::mat4 Projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.0f, 100.0f);
  // Camera matrix
  view_ = glm::lookAt(
            glm::vec3(0, 0, 5), // Camera center
            glm::vec3(0, 0, 0), // Look at point
            glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
  );
  if (reset_model)
  {
    // Model matrix : an identity matrix (model will be at the origin)
    model_ = glm::mat4(DEFAULT_MODEL_SCALE);
//    model_ *= glm::rotate(glm::radians(-45.0f), glm::vec3(1, 0, 0));
  }
}

void Visualizer::updateMatricesFromInputs()
{
  // TODO: Should be done per frame for everyone
  static double last_time = glfwGetTime();
  double current_time = glfwGetTime();
  double deltaTime = current_time - last_time;

  // TODO: These should probably also be members
  static double last_cursor_pos_x;
  static double last_cursor_pos_y;
  static glm::vec3 start_arcball_vector;
  static glm::mat4 start_model_matrix;

  int left_button_state = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT);
  int right_button_state = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT);

  double cursor_pos_x, cursor_pos_y;
  glfwGetCursorPos(window_, &cursor_pos_x, &cursor_pos_y);

  // TODO: Clean this up
  if (right_button_state == GLFW_PRESS && left_button_state != GLFW_PRESS)
  {
    if (!right_button_pressed_)
    {
      right_button_pressed_ = true;
      last_cursor_pos_x = cursor_pos_x;
      last_cursor_pos_y = cursor_pos_y;
    }
    double d_cursor_pos_x = cursor_pos_x - last_cursor_pos_x;
    double d_cursor_pos_y = -(cursor_pos_y - last_cursor_pos_y);
//    std::cout << "d_cursor_pos_x: " << d_cursor_pos_x << ", d_cursor_pos_y: " << d_cursor_pos_y << std::endl;
    d_cursor_pos_x /= width_;
    d_cursor_pos_y /= height_;

    // TODO: Should be a member
    float model_scale = glm::length(model_[0]);
    float translate_speed = 1.0f / model_scale / model_scale;
    glm::mat4 inv_model = glm::transpose(model_);
    glm::vec4 translation_vec = inv_model * glm::vec4(d_cursor_pos_x, d_cursor_pos_y, 0, 1) * translate_speed;
//    std::cout << "translation_vec: " << translation_vec << std::endl;
    model_ *= glm::translate(glm::vec3(translation_vec));

    last_cursor_pos_x = cursor_pos_x;
    last_cursor_pos_y = cursor_pos_y;
  }
  else if (left_button_state == GLFW_PRESS && right_button_state != GLFW_PRESS)
  {
    if (!left_button_pressed_)
    {
      left_button_pressed_ = true;
      last_cursor_pos_x = cursor_pos_x;
      last_cursor_pos_y = cursor_pos_y;
      start_arcball_vector = getArcballVector(cursor_pos_x, cursor_pos_y);
      start_model_matrix = model_;
      // TODO:
      arcball1_so_.setTransformation(glm::translate(start_arcball_vector * 5.0f) * glm::scale(glm::vec3(0.5, 0.5, 0.5)));
      arcball1_so_.setVisible(true);
    }
    // TODO: still some bug here
    glm::vec3 arcball_vector = getArcballVector(cursor_pos_x, cursor_pos_y);
    if (glm::length(arcball_vector - start_arcball_vector) > ARCBALL_ROTATION_THRESHOLD)
    {
      std::cout << "start arcball_vector: " << start_arcball_vector << std::endl;
      std::cout << "arcball_vector: " << arcball_vector << std::endl;
      float angle = std::acos(std::min(1.0f, glm::dot(start_arcball_vector, arcball_vector)));
      glm::vec3 rotation_axis = glm::cross(start_arcball_vector, arcball_vector);
      std::cout << "rotation_axis: " << rotation_axis << std::endl;
      glm::vec3 model_rotation_axis = glm::vec3(glm::transpose(start_model_matrix) * glm::vec4(rotation_axis, 1));
      model_ = glm::rotate(start_model_matrix, angle, model_rotation_axis);
    }
  }
  else
  {
    left_button_pressed_ = false;
    right_button_pressed_ = false;
  }

  last_time = current_time;
}

void Visualizer::run()
{
  if (window_ == nullptr)
  {
    throw std::runtime_error("Visualizer has not been initialized");
  }

  resetView();

  glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
  do
  {
    mvp_ = projection_  * view_ * model_;
    updateMatricesFromInputs();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
//    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    render();

    // Swap buffers
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }
  // Check if the ESC key was pressed or the window was closed
  while (!glfwWindowShouldClose(window_));

  glfwTerminate();
}

void Visualizer::render()
{
  octomap_so_.render(mvp_);
  so_.render(mvp_);
  arcball1_so_.render(mvp_);
  arcball2_so_.render(mvp_);
}
