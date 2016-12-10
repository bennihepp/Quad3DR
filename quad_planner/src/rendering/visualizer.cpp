//==================================================
// visualizer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#include <iostream>
#include <memory>
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
#include <quad_planner/rendering/lines.h>

using namespace glm;

namespace quad_planner
{
namespace rendering
{

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
: window_(nullptr), width_(0), height_(0), vertex_array_id_(0),
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

// Only for debugging
void APIENTRY glDebugOutput(GLenum source,
                            GLenum type,
                            GLuint id,
                            GLenum severity,
                            GLsizei length,
                            const GLchar *message,
                            const void *userParam)
{
    // ignore non-significant error/warning codes
    if(id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

    std::cout << "---------------" << std::endl;
    std::cout << "Debug message (" << id << "): " <<  message << std::endl;

    switch (source)
    {
        case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
        case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
    } std::cout << std::endl;

    switch (type)
    {
        case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
        case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
        case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
        case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
        case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
    } std::cout << std::endl;

    switch (severity)
    {
        case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
        case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
        case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
    } std::cout << std::endl;
    std::cout << std::endl;
}

void Visualizer::init(int core_minor, int core_major)
{
  const bool use_debug_context = false;

  if (!glfwInit())
  {
    throw std::runtime_error("Unable to initialize GLFW");
  }

  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, core_major); // We want OpenGL 3.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, core_minor);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //We don't want the old OpenGL
  // Only for debugging
  if (use_debug_context) {
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
  }

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

  if (use_debug_context) {
    GLint flags;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
      glEnable(GL_DEBUG_OUTPUT);
      glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
      glDebugMessageCallback(glDebugOutput, nullptr);
      glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
    }
    else {
      throw std::runtime_error("Unable to create OpenGL debug context");
    }
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
//  octomap_shader_ptr_->compileShader(ELEVATION_VERTEX_SHADER, ShaderProgram::VERTEX);
//  octomap_shader_ptr_->compileShader(SIMPLE_FRAGMENT_SHADER, ShaderProgram::FRAGMENT);
  octomap_shader_ptr_->compileShaderFromFile("quad_planner/shaders/octomap_shader.vertex", ShaderProgram::VERTEX);
  octomap_shader_ptr_->compileShaderFromFile("quad_planner/shaders/octomap_shader.geometry", ShaderProgram::GEOMETRY);
  octomap_shader_ptr_->compileShaderFromFile("quad_planner/shaders/octomap_shader.fragment", ShaderProgram::FRAGMENT);
//  octomap_shader_ptr_->compileShaderFromFile("shaders/simple_vertex_shader.vertexshader", ShaderProgram::VERTEX);
//  octomap_shader_ptr_->compileShaderFromFile("shaders/simple_fragment_shader.fragmentshader", ShaderProgram::FRAGMENT);
  octomap_shader_ptr_->link();

  arcball_shader_ptr_ = std::make_shared<ShaderProgram>();
//  arcball_shader_ptr_->compileShader(ELEVATION_VERTEX_SHADER, ShaderProgram::VERTEX);
//  arcball_shader_ptr_->compileShader(SIMPLE_FRAGMENT_SHADER, ShaderProgram::FRAGMENT);
  arcball_shader_ptr_->compileShaderFromFile("quad_planner/shaders/phong_shader.vertex", ShaderProgram::VERTEX);
  arcball_shader_ptr_->compileShaderFromFile("quad_planner/shaders/phong_shader.fragment", ShaderProgram::FRAGMENT);
  arcball_shader_ptr_->link();

  octomap_so_ = SceneObject(std::make_shared<OctomapRenderer>(), octomap_shader_ptr_);
  octomap_so_.setVisible(true);

  std::shared_ptr<ColorTriangleMesh> sphere_ptr = ColorTriangleMesh::createUnitSphere(ColorEigen::red(), 3);
  std::shared_ptr<ColorTriangleMesh> sphere_ptr2 = ColorTriangleMesh::createUnitSphere(ColorEigen::blue(), 3);
  std::shared_ptr<ColorTriangleMesh> sphere_ptr3 = ColorTriangleMesh::createUnitSphere(ColorEigen::green(), 3);
  so_ = SceneObject(sphere_ptr, arcball_shader_ptr_);
  so_.setTransformation(glm::translate(glm::vec3(0, 0, 1)));
  so_.setVisible(true);

  double cylinder_length = 10;
//  std::shared_ptr<ColorTriangleMesh> cylinder_ptr = ColorTriangleMesh::createUnitSphere(ColorEigen::green(), 3);
  std::shared_ptr<ColorTriangleMesh> cylinder_ptr = ColorTriangleMesh::createUnitRadiusCylinder(cylinder_length, ColorEigen::green(), 3);
  so2_ = SceneObject(cylinder_ptr, arcball_shader_ptr_);
  so2_.setTransformation(glm::translate(glm::vec3(0, 1, 2)) * glm::scale(glm::vec3(0.1)));
  so2_.setVisible(true);

  arcball1_so_.setShaderProgram(arcball_shader_ptr_);
  arcball1_so_.setRenderObject(sphere_ptr2);

  arcball2_so_.setShaderProgram(arcball_shader_ptr_);
  arcball2_so_.setRenderObject(sphere_ptr3);

  trajectory_shader_ptr_ = std::make_shared<ShaderProgram>();
  trajectory_shader_ptr_->compileShaderFromFile("quad_planner/shaders/trajectory_shader.vertex", ShaderProgram::VERTEX);
  trajectory_shader_ptr_->compileShaderFromFile("quad_planner/shaders/trajectory_shader.geometry", ShaderProgram::GEOMETRY);
  trajectory_shader_ptr_->compileShaderFromFile("quad_planner/shaders/trajectory_shader.fragment", ShaderProgram::FRAGMENT);
  trajectory_shader_ptr_->link();
  trajectory_shader_ptr_->use();
  trajectory_shader_ptr_->setUniform("lineWidth", 0.03f);
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

  float view_scale = glm::length(view_[0]);
  float translate_speed = 0.1f / view_scale / view_scale;
  glm::mat4 inv_view = glm::transpose(view_);
  glm::vec4 translation_vec = inv_view * glm::vec4(0, 0, translate_speed * yoffset, 1);
  view_ *= glm::translate(glm::vec3(translation_vec));
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

void Visualizer::resetView(bool reset_scale /*= true*/)
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
  if (reset_scale)
  {
    // Model matrix : an identity matrix (model will be at the origin)
    view_ *= glm::mat4(DEFAULT_MODEL_SCALE);
//    view_ *= glm::rotate(glm::radians(-45.0f), glm::vec3(1, 0, 0));
  }
}

void Visualizer::updateMatricesFromInputs()
{
  // TODO: Should be done per frame for everyone
//  static double last_time = glfwGetTime();
//  double current_time = glfwGetTime();
//  double deltaTime = current_time - last_time;

  // TODO: These should probably also be members
  static double last_cursor_pos_x;
  static double last_cursor_pos_y;
  static glm::vec3 start_arcball_vector;
  static glm::mat4 start_view_matrix;

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
    float view_scale = glm::length(view_[0]);
    float translate_speed = 1.0f / view_scale / view_scale;
    glm::mat4 inv_view = glm::transpose(view_);
    glm::vec4 translation_vec = inv_view * glm::vec4(d_cursor_pos_x, d_cursor_pos_y, 0, 1) * translate_speed;
//    std::cout << "translation_vec: " << translation_vec << std::endl;
    view_ *= glm::translate(glm::vec3(translation_vec));

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
      start_view_matrix = view_;
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
      glm::vec3 view_rotation_axis = glm::vec3(glm::transpose(start_view_matrix) * glm::vec4(rotation_axis, 1));
      view_ = glm::rotate(start_view_matrix, angle, view_rotation_axis);
    }
  }
  else
  {
    left_button_pressed_ = false;
    right_button_pressed_ = false;
  }

//  last_time = current_time;
}

void Visualizer::run(std::shared_ptr<ob::ProblemDefinition> pdef)
{
  if (window_ == nullptr)
  {
    throw std::runtime_error("Visualizer has not been initialized");
  }

  using StateSpaceT = ob::SE3StateSpace;
  const ob::PathPtr path = pdef->getSolutionPath();
  const og::PathGeometric *geo_path = dynamic_cast<og::PathGeometric*>(path.get());
  std::vector<SceneObject> objects;
  for (size_t i = 1; i < geo_path->getStateCount(); ++i) {
    auto state1 = dynamic_cast<const StateSpaceT::StateType*>(geo_path->getState(i-1));
    auto state2 = dynamic_cast<const StateSpaceT::StateType*>(geo_path->getState(i));
//    auto state1_pos = state1->as<ompl::base::RealVectorStateSpace::StateType>(0);
//    auto state2_pos = state2->as<ompl::base::RealVectorStateSpace::StateType>(0);
    Eigen::Vector3d state1_pos_vec(state1->getX(), state1->getY(), state1->getZ());
    Eigen::Vector3d state2_pos_vec(state2->getX(), state2->getY(), state2->getZ());
    auto lines = ColorLines3D::createLine(state1_pos_vec, state2_pos_vec, ColorEigen::yellow());
    SceneObject so(lines, trajectory_shader_ptr_);
    objects.push_back(so);
  }

  std::vector<std::shared_ptr<ColorLines3D>> lines;
//  double line_length = 10;
  auto p0 = Eigen::Vector3d(0, 0, .15);
  auto p1 = Eigen::Vector3d(-4.73269, -0.200741, 1.3174);
  auto p2 = Eigen::Vector3d(-6.88227, -0.0225858, 5.4814);
  auto p3 = Eigen::Vector3d(-3.79652, -1.11825, 5.05384);
  auto p4 = Eigen::Vector3d(-3.34997, -4.94136, 7.1635);
  auto p5 = Eigen::Vector3d(-1.64127, -7.5216, 4.01971);
  auto p6 = Eigen::Vector3d(0, -10, 1);
  lines.push_back(ColorLines3D::createLine(p0, p1, ColorEigen::magenta()));
  lines.push_back(ColorLines3D::createLine(p1, p2, ColorEigen::magenta()));
  lines.push_back(ColorLines3D::createLine(p2, p3, ColorEigen::magenta()));
  lines.push_back(ColorLines3D::createLine(p3, p4, ColorEigen::magenta()));
  lines.push_back(ColorLines3D::createLine(p4, p5, ColorEigen::magenta()));
  lines.push_back(ColorLines3D::createLine(p5, p6, ColorEigen::magenta()));
  std::vector<SceneObject> objects2;
  for (const auto& line : lines) {
    SceneObject so(line, trajectory_shader_ptr_);
    objects2.push_back(so);
  }

  auto point1 = Eigen::Vector3d(0, 0, 0);
  auto point2 = Eigen::Vector3d(4, 4, 1);
  std::shared_ptr<ColorLines3D> lines_ptr = ColorLines3D::createLine(point1, point2, ColorEigen::magenta());
  so2_ = SceneObject(lines_ptr, trajectory_shader_ptr_);
  so2_.setTransformation(glm::translate(glm::vec3(0, 1, 2)));
  so2_.setVisible(true);

  resetView();

  glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
  do
  {
    updateMatricesFromInputs();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
//    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // Enable alpha blendingglEnable( GL_BLEND )
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    render();
    for (const auto& object : objects) {
      object.render(view_, projection_, width_, height_);
    }
    for (const auto& object : objects2) {
      object.render(view_, projection_, width_, height_);
    }

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
  octomap_so_.render(view_, projection_, width_, height_);
  so_.render(view_, projection_, width_, height_);
  so2_.render(view_, projection_, width_, height_);
  arcball1_so_.render(view_, projection_, width_, height_);
  arcball2_so_.render(view_, projection_, width_, height_);
}


// Shaders
const char *Visualizer::SIMPLE_VERTEX_SHADER =
"#version 330 core\n"

"layout(location = 0) in vec3 vertexPosition_modelspace;\n"
"layout(location = 1) in vec3 vertexColor;\n"  // Output data
"out vec4 fragmentColor;\n"
"uniform mat4 mvp;\n"

"void main()\n"
"{\n"
"  gl_Position = mvp * vec4(vertexPosition_modelspace, 1);\n"
"  fragmentColor = vertexColor;\n"
"  //fragmentColor = vec4(0.5f, 0.2f * vertexPosition_modelspace[2], 0.5f, 1.0f);\n"
"}\n";

// Shaders
const char *Visualizer::ELEVATION_VERTEX_SHADER =
"#version 330 core\n"

"layout(location = 0) in vec3 vertexPosition_modelspace;\n"
"layout(location = 1) in vec3 vertexColor;\n"  // Output data
"out vec4 fragmentColor;\n"
"uniform mat4 mvp;\n"

"void main()\n"
"{\n"
"  gl_Position = mvp * vec4(vertexPosition_modelspace, 1);\n"
"  fragmentColor = vec4(0.5f, 0.2f * vertexPosition_modelspace[2], 0.5f, 1.0f);\n"
"}\n";

const char *Visualizer::SIMPLE_FRAGMENT_SHADER =
"#version 330 core\n"

"in vec4 fragmentColor;\n"
"out vec4 color;\n"  // Output data

"void main()\n"
"{\n"
"  color = fragmentColor;  // Output color = red\n"
"}\n";

}
}
