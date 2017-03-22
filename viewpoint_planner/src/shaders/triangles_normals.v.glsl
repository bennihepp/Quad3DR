#version 330

uniform mat4 u_pvm_matrix;

in vec3 a_position;
in vec3 a_normal;
in vec4 a_color;
out vec3 v_normal;

void main(void) {
  gl_Position = u_pvm_matrix * vec4(a_position, 1);
  v_normal = a_normal.xyz;
}
