#version 150

uniform mat4 u_pvm_matrix;

in vec3 a_position;
in vec4 a_color;
out vec4 v_color;

void main(void) {
  gl_Position = u_pvm_matrix * vec4(a_position, 1);
  v_color = a_color;
}
