#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;
layout(location = 2) in vec3 vertexColor;

// Uniform values
uniform float lineWidth;
uniform float viewportWidth;
uniform float viewportHeight;
uniform mat4 mvp;

out vec4 fragmentColor;

void main(){

    gl_Position = mvp * vec4(vertexPosition_modelspace, 1);
    fragmentColor = vec4(0.5f, 0.2f * vertexPosition_modelspace[2], 0.5f, 1.0f);
}
