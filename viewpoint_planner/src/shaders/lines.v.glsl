#version 330

uniform mat4 u_pvm_matrix;

in vec3 a_pos;
in vec4 a_color;
out vec4 v_pos;
out vec4 v_color;

void main(void) {
  v_pos = u_pvm_matrix * vec4(a_pos, 1);
  v_color = a_color;
}
