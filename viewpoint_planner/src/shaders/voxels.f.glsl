#version 330

// INPUTS
in vec3 v_light_position_cameraspace;
in vec4 v_color;
in vec3 v_vertex_position_cameraspace;
in vec3 v_vertex_normal_cameraspace;
//in vec4 v_vertex_position;
//in vec3 v_vertex_normal;
//in vec3 v_light_direction;
//in vec3 v_light_position;
flat in int v_keep_flag;

// OUTPUTS
out vec4 f_color;

const vec3 light_color = vec3(1, 1, 1);

void main() {
  // Light emission properties
//  float u_light_power = 5000.0f;

  if (v_keep_flag == 0) {
    discard;
  }

  // Material properties
  vec3 material_color = v_color.xyz;
  vec3 u_ambient_color = 0.4 * material_color;
  vec3 u_diffuse_color = 0.7 * material_color;
  vec3 u_specular_color = 0.5 * material_color;
  float u_specular_alpha = 3;

  // Distance to the light
  vec3 light_direction_cameraspace = v_light_position_cameraspace - v_vertex_position_cameraspace;
  float distance = length(light_direction_cameraspace);
  // Direction of fragment
  vec3 eye_direction_cameraspace = - v_vertex_position_cameraspace;

  // Normal of the computed fragment, in camera space
  vec3 N = normalize(v_vertex_normal_cameraspace);
  // Direction of the light (from the fragment to the light)
  vec3 L = normalize(light_direction_cameraspace);
  // Cosine of the angle between the normal and the light direction,
  // clamped above 0
  //  - light is at the vertical of the triangle -> 1
  //  - light is perpendicular to the triangle -> 0
  //  - light is behind the triangle -> 0
  float lambertian = clamp(dot(N, L), 0, 1);

  float specular = 0;
  if (lambertian > 0) {
    // Light ray vector (towards the camera)
    vec3 E = normalize(eye_direction_cameraspace);
    // Direction in which the triangle reflects the light
    vec3 R = reflect(-L, N);
    // Cosine of the angle between the Eye vector and the Reflect vector,
    // clamped to 0
    //  - Looking into the reflection -> 1
    //  - Looking elsewhere -> < 1
    float cos_alpha = clamp(dot(E, R), 0, 1);
    specular = pow(cos_alpha, u_specular_alpha);
  }

//  float attenuated_light_power = u_light_power / (distance * distance);
  float attenuated_light_power = 1;
  f_color.rgb =
    u_ambient_color +
    lambertian * u_diffuse_color * light_color * attenuated_light_power +
    specular * u_specular_color * light_color * attenuated_light_power;
  f_color.a = v_color.a;
}
