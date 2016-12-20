#version 330 core

layout (lines) in;
//layout (line_strip, max_vertices = 10) out;
layout (triangle_strip, max_vertices = 6) out;

// Inputs
in vec3 fragmentColorGeom[2];

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
uniform float lineWidth;

// Outputs
out vec4 line_vertex1;
out vec4 line_normal;
out vec4 line_direction;
out vec3 fragmentColor;

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

void main()
{
	vec3 color1 = fragmentColorGeom[0];
	vec3 color2 = fragmentColorGeom[1];

	vec4 vertex1 = gl_in[0].gl_Position;
	vec4 vertex2 = gl_in[1].gl_Position;
	vec3 vertex1_2d = vec3_from_homogeneous(vertex1);
	vec3 vertex2_2d = vec3_from_homogeneous(vertex2);

	vec3 z_vec = vec3(0, 0, 1);
	vec3 direction_2d = vertex2_2d - vertex1_2d;
	vec3 normal_2d = vec3(-direction_2d.y, direction_2d.x, 0);
	//vec3 normal_2d = cross(z_vec, direction_2d);
	normal_2d = normal_2d / length(normal_2d);
	
	vec4 normal1 = vec3_to_homogeneous(normal_2d, vertex1.w);
	vec4 normal2 = vec3_to_homogeneous(normal_2d, vertex2.w);
	normal2 = normal1;

	float lineWidth1 = lineWidth / vertex1.z;
	float lineWidth2 = lineWidth / vertex2.z;

	line_vertex1 = vertex1;
	line_direction = vertex2 - vertex1;
    line_normal = normal1;

	//
	// Draw triangles
	//

    gl_Position.xyz = vertex1.xyz - lineWidth1 * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = color1;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz - lineWidth2 * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = color2;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz + lineWidth2 * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = color2;
    EmitVertex();

    EndPrimitive();

    gl_Position.xyz = vertex1.xyz - lineWidth1 * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = color1;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz + lineWidth2 * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = color2;
    EmitVertex();
    gl_Position.xyz = vertex1.xyz + lineWidth1 * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = color1;
    EmitVertex();

    EndPrimitive();

/*
	//
	// Draw wireframe
	//

    gl_Position = vertex1;
    fragmentColor = colorGreen;
    EmitVertex();
    gl_Position = vertex2;
    fragmentColor = colorGreen;
    EmitVertex();

    EndPrimitive();

    gl_Position.xyz = vertex1.xyz - lineWidth * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = colorBlue;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz - lineWidth * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = colorBlue;
    EmitVertex();

    EndPrimitive();

    gl_Position.xyz = vertex1.xyz + lineWidth * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = colorBlue;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz + lineWidth * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = colorBlue;
    EmitVertex();

    EndPrimitive();

    gl_Position.xyz = vertex1.xyz - lineWidth * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = colorRed;
    EmitVertex();
    gl_Position.xyz = vertex1.xyz + lineWidth * normal1.xyz;
    gl_Position.w = vertex1.w;
    fragmentColor = colorRed;
    EmitVertex();

    EndPrimitive();

    gl_Position.xyz = vertex2.xyz - lineWidth * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = colorRed;
    EmitVertex();
    gl_Position.xyz = vertex2.xyz + lineWidth * normal2.xyz;
    gl_Position.w = vertex2.w;
    fragmentColor = colorRed;
    EmitVertex();

    EndPrimitive();
*/
}
