#version 150

// UNIFORMS
uniform mat4 u_view_matrix;
uniform mat4 u_model_matrix;
uniform mat4 u_pvm_matrix;

uniform float u_alpha_override;
uniform vec3 u_light_position;
//uniform float2 viewport_size;

uniform samplerBuffer u_pos_texture;
uniform samplerBuffer u_offset_normal_texture;
uniform samplerBuffer u_color_texture;

// OUTPUTS
out vec4 v_color;
out vec3 v_light_position_cameraspace;
out vec3 v_vertex_position_cameraspace;
out vec3 v_vertex_normal_cameraspace;
//out vec4 v_vertex_position;
//out vec3 v_vertex_normal;
//out vec3 v_light_direction;
//out vec3 v_light_position;

const float VOXEL_SHRINK_EPS = 0;
const int VERTICES_PER_VOXEL = 12 * 3;

void computeVoxelOffsetIndex(out int voxel_index, out int offset_index) {
  voxel_index = gl_VertexID / VERTICES_PER_VOXEL;
  offset_index = gl_VertexID % VERTICES_PER_VOXEL;
}

vec4 computeVoxelOffsetVertex(int voxel_index, int offset_index) {
  vec4 texel = texelFetch(u_pos_texture, voxel_index);

  vec3 voxel_position = texel.xyz;
  float voxel_size = texel.a - VOXEL_SHRINK_EPS;

  vec3 vertex_offset = texelFetch(u_offset_normal_texture, offset_index).xyz;

  vec4 vertex_position = vec4(voxel_position.xyz + voxel_size * vertex_offset.xyz, 1.0);
  return vertex_position;
}

vec3 computeVoxelOffsetNormal(int offset_index) {
  vec3 vertex_normal = texelFetch(u_offset_normal_texture, VERTICES_PER_VOXEL + offset_index).xyz;
  return vertex_normal;
}

vec4 computeVoxelColor(int voxel_index) {
  //v_color = a_color;
  vec4 color = texelFetch(u_color_texture, voxel_index);
  color.a = u_alpha_override >= 0 ? u_alpha_override : color.a;
  return color;
}

vec4 transformVectorToCameraspace(mat4 view_model_matrix, vec4 vector) {
  vec4 vector_cameraspace = view_model_matrix * vector;
  return vector_cameraspace;
}

void main(void) {
  mat4 view_model_matrix = u_view_matrix * u_model_matrix;

  int voxel_index;
  int offset_index;
  computeVoxelOffsetIndex(voxel_index, offset_index);

  vec4 vertex_position = computeVoxelOffsetVertex(voxel_index, offset_index);
  vec3 vertex_normal = computeVoxelOffsetNormal(offset_index);

  v_vertex_position_cameraspace = transformVectorToCameraspace(view_model_matrix, vertex_position).xyz;
  v_vertex_normal_cameraspace = transformVectorToCameraspace(view_model_matrix, vec4(vertex_normal, 0)).xyz;
  v_light_position_cameraspace = transformVectorToCameraspace(view_model_matrix, vec4(u_light_position, 1)).xyz;

//  v_vertex_normal_cameraspace = (view_model_matrix * vec4(0, 0, 1, 1)).xyz;
//  v_light_position_cameraspace = (view_model_matrix * vec4(0, 0, 10, 1)).xyz;

  gl_Position = u_pvm_matrix * vertex_position;

  v_color = computeVoxelColor(voxel_index);

//  v_vertex_position = vertex_position;
//  v_vertex_normal = vertex_normal;
//  v_light_position = u_light_position;
//  v_light_direction = (u_light_position - vertex_position.xyz);
}
