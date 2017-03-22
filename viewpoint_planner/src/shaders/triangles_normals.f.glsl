#version 330

in vec3 v_normal;
out vec4 f_color;

void main(void) {
  f_color = vec4(0.5 * v_normal + vec3(0.5, 0.5, 0.5), 1);
  //mat3 normal_matrix = mat3(u_pvm_matrix);
  //normal_matrix = inverse(normal_matrix);
  //normal_matrix = transpose(normal_matrix);
  //vec3 transformed_normal = normal_matrix * a_normal;
  //f_color = vec4(-transformed_normal, 1);
}
