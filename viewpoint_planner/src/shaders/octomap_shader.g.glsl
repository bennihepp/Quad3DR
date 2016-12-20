#version 330 core

layout (points) in;
layout (triangle_strip, max_vertices = 36) out;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
//uniform vec3 light_position_worldspace;

// Inputs
in vec4 fragment_color_vs[1];
in vec3 position_modelspace_vs[1];
in float voxel_size_vs[1];
in vec3 light_position_worldspace_vs[1];

// Outputs
out vec4 fragment_color;
out vec3 position_worldspace;
out vec3 normal_cameraspace;
out vec3 eye_direction_cameraspace;
out vec3 light_direction_cameraspace;

// Compute position in normalized device coordinates: MVP * position
vec4 compute_position_ndc(vec3 position_modelspace)
{
    return MVP * vec4(position_modelspace, 1);
}

// Compute position of vertex in worldspace : M * position
vec3 compute_position_worldspace(vec3 position_modelspace)
{
	return (M * vec4(position_modelspace, 1)).xyz;
}

// Vector that goes from the vertex to the camera, in camera space.
// In camera space, the camera is at the origin (0,0,0).
vec3 compute_eye_direction_cameraspace(vec3 position_modelspace)
{
	vec3 position_cameraspace = (V * M * vec4(position_modelspace,1)).xyz;
	vec3 eye_direction_cameraspace = vec3(0, 0, 0) - position_cameraspace;
	return eye_direction_cameraspace;
}

// Vector that goes from the vertex to the light, in camera space. M is ommited because it's identity.
vec3 compute_light_direction_cameraspace(vec3 light_position_worldspace, vec3 eye_direction_cameraspace)
{
	vec3 light_position_cameraspace = (V * vec4(light_position_worldspace, 1)).xyz;
	vec3 light_direction_cameraspace = light_position_cameraspace + eye_direction_cameraspace;
	return light_direction_cameraspace;
}

// Normal in camera space
vec3 compute_normal_cameraspace(vec3 normal_modelspace)
{
	// Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.
	return normal_cameraspace = ( V * M * vec4(normal_modelspace,0)).xyz;
}

void main()
{
	//vec4 vertex1 = gl_in[0].gl_Position;
	//vec4 vertex2 = gl_in[1].gl_Position;
	//vec4 vertex3 = gl_in[2].gl_Position;

	vec3 light_position_worldspace = light_position_worldspace_vs[0];
	float voxel_size = voxel_size_vs[0];
	fragment_color = fragment_color_vs[0];

	vec3 position_modelspace;
	vec3 normal_modelspace;

	// Lower triangle 1
	normal_modelspace = vec3(0, 0, -1);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

	// Lower triangle 2
	normal_modelspace = vec3(0, 0, -1);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();


	// Upper triangle 1
	normal_modelspace = vec3(0, 0, +1);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

	// Upper triangle 2
	normal_modelspace = vec3(0, 0, +1);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();


	// Left (x) triangle 1
	normal_modelspace = vec3(-1, 0, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();

	// Left (x) triangle 2
	normal_modelspace = vec3(-1, 0, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();


	// Right (x) triangle 1
	normal_modelspace = vec3(+1, 0, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();

	// Right (x) triangle 2
	normal_modelspace = vec3(+1, 0, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();


	// Back (+y) triangle 1
	normal_modelspace = vec3(0, +1, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();

	// Back (+y) triangle 2
	normal_modelspace = vec3(0, +1, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, +0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();


	// Front (-y) triangle 1
	normal_modelspace = vec3(0, -1, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();

	// Front (-y) triangle 2
	normal_modelspace = vec3(0, -1, 0);
	normal_cameraspace = compute_normal_cameraspace(normal_modelspace);
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, -0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(+0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();
	//
	position_modelspace = position_modelspace_vs[0] + vec3(-0.5 * voxel_size, -0.5 * voxel_size, +0.5 * voxel_size);
	//
    gl_Position = compute_position_ndc(position_modelspace);
	position_worldspace = compute_position_worldspace(position_modelspace);
	eye_direction_cameraspace = compute_eye_direction_cameraspace(position_modelspace);
	light_direction_cameraspace = compute_light_direction_cameraspace(light_position_worldspace, eye_direction_cameraspace);
	//
    EmitVertex();

    EndPrimitive();
}

/*layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
//uniform vec3 lightPosition_worldspace;

// Inputs
//in vec4 fragment_color_vs[3];
in vec3 position_modelspace_vs[3];
in vec3 position_worldspace_vs[3];
in vec3 normal_cameraspace_vs[3];
in vec3 eyeDirection_cameraspace_vs[3];
in vec3 lightDirection_cameraspace_vs[3];

// Outputs
//out vec4 fragment_color;
out vec3 position_modelspace;
out vec3 position_worldspace;
out vec3 normal_cameraspace;
out vec3 eyeDirection_cameraspace;
out vec3 lightDirection_cameraspace;

void main()
{
	//vec4 vertex1 = gl_in[0].gl_Position;
	//vec4 vertex2 = gl_in[1].gl_Position;
	//vec4 vertex3 = gl_in[2].gl_Position;

    gl_Position = gl_in[0].gl_Position;
	position_modelspace = position_modelspace_vs[0];
	position_worldspace = position_worldspace_vs[0];
	normal_cameraspace = normal_cameraspace_vs[0];
	eyeDirection_cameraspace = eyeDirection_cameraspace_vs[0];
	lightDirection_cameraspace = lightDirection_cameraspace_vs[0];
    EmitVertex();
    gl_Position = gl_in[1].gl_Position;
	position_modelspace = position_modelspace_vs[1];
	position_worldspace = position_worldspace_vs[1];
	normal_cameraspace = normal_cameraspace_vs[1];
	eyeDirection_cameraspace = eyeDirection_cameraspace_vs[1];
	lightDirection_cameraspace = lightDirection_cameraspace_vs[1];
    EmitVertex();
    gl_Position = gl_in[2].gl_Position;
	position_modelspace = position_modelspace_vs[2];
	position_worldspace = position_worldspace_vs[2];
	normal_cameraspace = normal_cameraspace_vs[2];
	eyeDirection_cameraspace = eyeDirection_cameraspace_vs[2];
	lightDirection_cameraspace = lightDirection_cameraspace_vs[2];
    EmitVertex();

    EndPrimitive();
}
*/