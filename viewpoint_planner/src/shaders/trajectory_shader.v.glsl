#version 330 core

// Inputs
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexColor;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
uniform float lineWidth;

// Outputs
out vec3 fragmentColorGeom;

void main()
{
    gl_Position = MVP * vec4(vertexPosition_modelspace, 1);

    fragmentColorGeom = vertexColor;
}
