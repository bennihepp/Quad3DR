#version 330

uniform mat4 u_pvm_matrix;

in vec3 a_position;
in vec3 a_normal;
in vec4 a_color;
out vec4 v_color;

void main(void) {
  gl_Position = u_pvm_matrix * vec4(a_position, 1);
  //v_color = a_color;
  v_color = vec4(0.5 * a_normal.xyz + vec3(0.5, 0.5, 0.5), 1);
  //mat3 normal_matrix = mat3(u_pvm_matrix);
  //normal_matrix = inverse(normal_matrix);
  //normal_matrix = transpose(normal_matrix);
  //vec3 transformed_normal = normal_matrix * a_normal;
  //v_color = vec4(-transformed_normal, 1);
}
