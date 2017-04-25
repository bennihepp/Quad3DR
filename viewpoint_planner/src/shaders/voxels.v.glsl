#version 330

// UNIFORMS
uniform mat4 u_vm_matrix;
uniform mat4 u_pvm_matrix;

uniform float u_voxel_size_factor;
uniform uint u_color_mode;
uniform float u_weight_color_scale;
uniform float u_weight_color_offset;
uniform float u_observation_count_color_scale;
uniform float u_observation_count_color_offset;
uniform float u_information_color_scale;
uniform float u_information_color_offset;
uniform float u_alpha_override;
uniform vec3 u_light_position;
//uniform float2 viewport_size;

uniform float u_min_voxel_size;
uniform float u_max_voxel_size;
uniform float u_min_occupancy;
uniform float u_max_occupancy;
uniform float u_min_observations;
uniform float u_max_observations;
uniform float u_min_weight;
uniform float u_max_weight;
uniform float u_min_information;
uniform float u_max_information;

uniform samplerBuffer u_pos_texture;
uniform samplerBuffer u_offset_normal_texture;
uniform samplerBuffer u_color_texture;
uniform samplerBuffer u_info_texture;

// OUTPUTS
out vec4 v_color;
out vec3 v_light_position_cameraspace;
out vec3 v_vertex_position_cameraspace;
out vec3 v_vertex_normal_cameraspace;
//out vec4 v_vertex_position;
//out vec3 v_vertex_normal;
//out vec3 v_light_direction;
//out vec3 v_light_position;
flat out int v_keep_flag;
flat out int v_do_shading;

const int VERTICES_PER_VOXEL = 12 * 3;

// Color flags
const uint COLOR_FIXED = 1u << 1u;
const uint COLOR_WEIGHT = 1u << 3u;
const uint COLOR_OCCUPANCY = 1u << 4u;
const uint COLOR_OBSERVATION_COUNT = 1u << 5u;
const uint COLOR_UNKNOWN_LOW_ALPHA = 1u << 6u;
const uint COLOR_INFORMATION = 1u << 7u;
const uint COLOR_INDEX = 1u << 8u;

void computeVoxelOffsetIndex(out int voxel_index, out int offset_index) {
  voxel_index = gl_VertexID / VERTICES_PER_VOXEL;
  offset_index = gl_VertexID % VERTICES_PER_VOXEL;
}

vec4 computeVoxelOffsetVertex(int voxel_index, int offset_index, out float voxel_size) {
  vec4 texel = texelFetch(u_pos_texture, voxel_index);

  vec3 voxel_position = texel.xyz;
  voxel_size = texel.a * u_voxel_size_factor;

  vec3 vertex_offset = texelFetch(u_offset_normal_texture, offset_index).xyz;

  vec4 vertex_position = vec4(voxel_position.xyz + voxel_size * vertex_offset.xyz, 1.0);
  return vertex_position;
}

vec3 computeVoxelOffsetNormal(int offset_index) {
  vec3 vertex_normal = texelFetch(u_offset_normal_texture, VERTICES_PER_VOXEL + offset_index).xyz;
  return vertex_normal;
}

vec4 computeVoxelColor(int voxel_index) {
  vec4 color = texelFetch(u_color_texture, voxel_index);
  return color;
}

void computeVoxelInfo(int voxel_index, out float occupancy, out float observation_count, out float weight, out float information) {
  occupancy = texelFetch(u_info_texture, voxel_index).x;
  observation_count = texelFetch(u_info_texture, voxel_index).y;
  weight = texelFetch(u_info_texture, voxel_index).z;
  information = texelFetch(u_info_texture, voxel_index).w;
}

vec4 transformVectorToCameraspace(mat4 view_model_matrix, vec4 vector) {
  vec4 vector_cameraspace = view_model_matrix * vector;
  return vector_cameraspace;
}

vec4 indexToColor(int index) {
    int r = (index & 0x000000FF) >> 0;
    int g = (index & 0x0000FF00) >> 8;
    int b = (index & 0x00FF0000) >> 16;
	vec4 color;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
	color.a = 1;
	return color;
}

void main(void) {
  mat4 view_model_matrix = u_vm_matrix;

  // Retrieve voxel and offset index
  int voxel_index;
  int offset_index;
  computeVoxelOffsetIndex(voxel_index, offset_index);

  // Retrieve voxel position and offset and additional info
  float voxel_size;
  vec4 vertex_position = computeVoxelOffsetVertex(voxel_index, offset_index, voxel_size);
  vec3 vertex_normal = computeVoxelOffsetNormal(offset_index);
  float occupancy;
  float observation_count;
  float weight;
  float information;
  computeVoxelInfo(voxel_index, occupancy, observation_count, weight, information);

  // Filter voxels based on size, occupancy and observation count
  v_keep_flag = 1;
  v_keep_flag = voxel_size >= u_min_voxel_size ? v_keep_flag : 0;
  v_keep_flag = voxel_size <= u_max_voxel_size ? v_keep_flag : 0;
  v_keep_flag = occupancy >= u_min_occupancy ? v_keep_flag : 0;
  v_keep_flag = occupancy <= u_max_occupancy ? v_keep_flag : 0;
  v_keep_flag = observation_count >= u_min_observations ? v_keep_flag : 0;
  v_keep_flag = observation_count <= u_max_observations ? v_keep_flag : 0;
  v_keep_flag = weight >= u_min_weight ? v_keep_flag : 0;
  v_keep_flag = weight <= u_max_weight ? v_keep_flag : 0;
  v_keep_flag = information >= u_min_information ? v_keep_flag : 0;
  v_keep_flag = information <= u_max_information ? v_keep_flag : 0;

  v_vertex_position_cameraspace = transformVectorToCameraspace(view_model_matrix, vertex_position).xyz;
  v_vertex_normal_cameraspace = transformVectorToCameraspace(view_model_matrix, vec4(vertex_normal, 0)).xyz;
  v_light_position_cameraspace = transformVectorToCameraspace(view_model_matrix, vec4(u_light_position, 1)).xyz;
  // Let light move with camera
  //v_light_position_cameraspace = u_light_position;

//  v_vertex_normal_cameraspace = (view_model_matrix * vec4(0, 0, 1, 1)).xyz;
//  v_light_position_cameraspace = (view_model_matrix * vec4(0, 0, 10, 1)).xyz;

  gl_Position = u_pvm_matrix * vertex_position;

  v_color = computeVoxelColor(voxel_index);
  v_do_shading = 1;

  // Apply color flags
  if ((u_color_mode & COLOR_WEIGHT) != 0u) {
    float weight_scaled = 0.5 * u_weight_color_scale * (weight - u_weight_color_offset);
    //v_color = vec4(1 - weight_scaled, weight_scaled, 0, 1);
    v_color = vec4(0.5 - weight_scaled, 0.5 + weight_scaled, 0.5 - weight_scaled, 1);
  }
  if ((u_color_mode & COLOR_OCCUPANCY) != 0u) {
    v_color = vec4(1 - occupancy, 0, occupancy, 1);
  }
  if ((u_color_mode & COLOR_OBSERVATION_COUNT) != 0u) {
    float observation_count_scaled = u_observation_count_color_scale * (observation_count - u_observation_count_color_offset);
    v_color = vec4(1 - observation_count_scaled, 0, observation_count_scaled, 1);
  }
  if ((u_color_mode & COLOR_INFORMATION) != 0u) {
    float information_scaled = 0.5 * u_information_color_scale * (information - u_information_color_offset);
    //v_color = vec4(1 - information_scaled, information_scaled, 0, 1);
    v_color = vec4(0.5 - information_scaled, 0.5 + information_scaled, 0.5 - information_scaled, 1);
  }
  // TODO: Coloring of unknown voxels. Make configurable with uniforms.
  if ((u_color_mode & COLOR_UNKNOWN_LOW_ALPHA) != 0u) {
    if (observation_count == 0) {
      v_color = vec4(1, 1, 0, 0.2);
    }
  }
  // TODO: Coloring of unknown voxels. Make configurable with uniforms.
  if ((u_color_mode & COLOR_INDEX) != 0u) {
    v_color = indexToColor(voxel_index);
    v_do_shading = 0;
  }
  else {
    v_color.a = u_alpha_override >= 0 ? u_alpha_override : v_color.a;
  }

//  v_vertex_position = vertex_position;
//  v_vertex_normal = vertex_normal;
//  v_light_position = u_light_position;
//  v_light_direction = (u_light_position - vertex_position.xyz);
}
