#version 330

uniform mat4 u_pvm_matrix;
uniform mat4 u_vm_matrix;

in vec3 a_position;
in vec4 a_color;
out float v_dist_to_camera;

void main(void) {
  vec4 cs_position = u_vm_matrix * vec4(a_position, 1);
  v_dist_to_camera = -cs_position.z * cs_position.w;
  gl_Position = u_pvm_matrix * vec4(a_position, 1);
}
