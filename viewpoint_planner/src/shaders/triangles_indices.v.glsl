#version 330

uniform mat4 u_pvm_matrix;
uniform int u_index_offset;

in vec3 a_position;
in vec4 a_color;
out vec4 v_color;

const int VERTICES_PER_PRIMITIVE = 3;

int getPrimitiveIndex() {
  int index = u_index_offset + gl_VertexID / VERTICES_PER_PRIMITIVE;
  return index;
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
  int index = getPrimitiveIndex();

  gl_Position = u_pvm_matrix * vec4(a_position, 1);
  v_color = indexToColor(index);
}
