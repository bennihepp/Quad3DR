#version 330

in float v_dist_to_camera;
out vec4 f_color;

vec3 unpackColor3(float f) {
	vec3 color;
	color.r = floor(f / 256.);
	color.g = floor(f - color.r * 256.);
	color.b = floor((f - color.r * 256. - color.g) * 256.);
	return color / 256.;
}

void main(void) {
	f_color = vec4(unpackColor3(v_dist_to_camera), 1);
}
