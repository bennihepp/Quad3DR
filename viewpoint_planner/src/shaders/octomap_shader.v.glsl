#version 330 core

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertex_position_modelspace;
layout(location = 1) in float voxel_size;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
//uniform vec3 light_position_worldspace;

// Outputs
out vec4 fragment_color_vs;
out vec3 position_modelspace_vs;
out float voxel_size_vs;
out vec3 light_position_worldspace_vs;

void main()
{
	voxel_size_vs = voxel_size;

	light_position_worldspace_vs = vec3(5, 5, 10);

	position_modelspace_vs = vertex_position_modelspace;

    gl_Position = MVP * vec4(vertex_position_modelspace, 1);

	// Position of the vertex, in worldspace : M * position
	vec3 position_worldspace = (M * vec4(vertex_position_modelspace, 1)).xyz;
	
	// Vector that goes from the vertex to the camera, in camera space.
	// In camera space, the camera is at the origin (0,0,0).
	//vec3 vertexPosition_cameraspace = (V * M * vec4(vertexPosition_modelspace,1)).xyz;
	//eyeDirection_cameraspace_vs = vec3(0,0,0) - vertexPosition_cameraspace;

	// Vector that goes from the vertex to the light, in camera space. M is ommited because it's identity.
	//vec3 lightPosition_cameraspace = (V * vec4(lightPosition_worldspace, 1)).xyz;
	//lightDirection_cameraspace_vs = lightPosition_cameraspace + eyeDirection_cameraspace_vs;

	// Normal of the vertex, in camera space
	//normal_cameraspace_vs = ( V * M * vec4(vertexNormal_modelspace,0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.
	
	float color_level = 0.2f * position_worldspace[2];
	float green = color_level < 0.5f ? 2 * color_level : 2 - 2 * color_level;
	fragment_color_vs = vec4(clamp(1.0f - color_level, 0, 1), green, clamp(color_level, 0, 1), 1.0f);
}
