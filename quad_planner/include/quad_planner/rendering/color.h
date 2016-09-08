#pragma once

#include <quad_planner/rendering/linalg.h>

namespace quad_planner
{
namespace rendering
{

  class ColorGlm
  {
  public:
    static glm::vec3 red()
    {
      return glm::vec3(1.0f, 0.0f, 0.0f);
    }
    static glm::vec3 green()
    {
      return glm::vec3(0.0f, 1.0f, 0.0f);
    }
    static glm::vec3 blue()
    {
      return glm::vec3(0.0f, 0.0f, 1.0f);
    }
    static glm::vec3 yellow()
    {
      return glm::vec3(1.0f, 1.0f, 0.0f);
    }
    static glm::vec3 magenta()
    {
      return glm::vec3(1.0f, 0.0f, 1.0f);
    }
    static glm::vec3 cyan()
    {
      return glm::vec3(0.0f, 1.0f, 1.0f);
    }
    static glm::vec3 orange()
    {
      return glm::vec3(1.0f, 0.5f, 0.0f);
    }
    static glm::vec3 violet()
    {
      return glm::vec3(0.5f, 0.0f, 1.0f);
    }
    static glm::vec3 grey()
    {
      return glm::vec3(0.5f, 0.5f, 0.5f);
    }
    static glm::vec3 white()
    {
      return glm::vec3(1.0f, 1.0f, 1.0f);
    }
    static glm::vec3 black()
    {
      return glm::vec3(0.0f, 0.0f, 0.0f);
    }
  };

  class ColorEigen
  {
  public:
    static Eigen::Vector3d red()
    {
      return Eigen::Vector3d(1.0f, 0.0f, 0.0f);
    }
    static Eigen::Vector3d green()
    {
      return Eigen::Vector3d(0.0f, 1.0f, 0.0f);
    }
    static Eigen::Vector3d blue()
    {
      return Eigen::Vector3d(0.0f, 0.0f, 1.0f);
    }
    static Eigen::Vector3d yellow()
    {
      return Eigen::Vector3d(1.0f, 1.0f, 0.0f);
    }
    static Eigen::Vector3d magenta()
    {
      return Eigen::Vector3d(1.0f, 0.0f, 1.0f);
    }
    static Eigen::Vector3d cyan()
    {
      return Eigen::Vector3d(0.0f, 1.0f, 1.0f);
    }
    static Eigen::Vector3d orange()
    {
      return Eigen::Vector3d(1.0f, 0.5f, 0.0f);
    }
    static Eigen::Vector3d violet()
    {
      return Eigen::Vector3d(0.5f, 0.0f, 1.0f);
    }
    static Eigen::Vector3d grey()
    {
      return Eigen::Vector3d(0.5f, 0.5f, 0.5f);
    }
    static Eigen::Vector3d white()
    {
      return Eigen::Vector3d(1.0f, 1.0f, 1.0f);
    }
    static Eigen::Vector3d black()
    {
      return Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    }
  };

}
}
