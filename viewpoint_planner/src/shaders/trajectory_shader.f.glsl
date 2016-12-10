#version 330 core

// Input vertex data, different for all executions of this shader.

// Inputs
in vec4 line_vertex1;
in vec4 line_normal;
in vec4 line_direction;
in vec3 fragmentColor;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
uniform float lineWidth;

// Ouput data
out vec4 color;

vec4 vec3_to_homogeneous(vec3 vec, float w)
{
	vec4 vec_3d;
	vec_3d.xy = vec.xy;
	vec_3d.z = 0;
	vec_3d.w = 1;
	vec_3d *= w;
	return vec_3d;
}

vec3 vec3_from_homogeneous(vec4 vec)
{
	vec3 vec_2d = vec.xyz;
	vec_2d.z = 0;
	vec_2d /= vec.w;
	return vec_2d;
}

vec3 convertDeviceCoordToViewCoord(vec4 fragCoord)
{
	vec3 viewCoord = fragCoord.xyz;
	viewCoord.z = 0;
	viewCoord.x /= viewportWidth;
	viewCoord.y /= viewportHeight;
	viewCoord = viewCoord * 2.0 - 1.0;
	return viewCoord;
}

void main()
{
	vec3 line_vertexTemp_vec3 = convertDeviceCoordToViewCoord(gl_FragCoord);
	vec3 line_vertex1_vec3 = line_vertex1.xyz / line_vertex1.w;
	vec3 dvertex = line_vertexTemp_vec3 - line_vertex1_vec3;

	vec3 line_normal_vec3 = line_normal.xyz;
	vec3 line_direction_vec3 = normalize(line_direction.xyz);
	float dist = abs(dot(dvertex, line_normal_vec3) / lineWidth);
	float weight = 1.0 - dist;
	weight = sqrt(1.0 - dist);

	color.xyz = fragmentColor;
	color.w = clamp(weight, 0, 1);
}
