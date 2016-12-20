#version 330 core

// Inputs
in vec4 fragmentColor;
in vec3 position_modelspace;
in vec3 position_worldspace;
in vec3 normal_cameraspace;
in vec3 eyeDirection_cameraspace;
in vec3 lightDirection_cameraspace;

// Uniform values
uniform mat4 MVP;
uniform mat4 M;
uniform mat4 V;
uniform float viewportWidth;
uniform float viewportHeight;
//uniform vec3 lightPosition_worldspace;

// parameters of the light and possible values
//uniform vec3 u_lightAmbientIntensitys; // = vec3(0.6, 0.3, 0);
//uniform vec3 u_lightDiffuseIntensitys; // = vec3(1, 0.5, 0);
//uniform vec3 u_lightSpecularIntensitys; // = vec3(0, 1, 0);

// parameters of the material and possible values
//uniform vec3 u_matAmbientReflectances; // = vec3(1, 1, 1);
//uniform vec3 u_matDiffuseReflectances; // = vec3(1, 1, 1);
//uniform vec3 u_matSpecularReflectances; // = vec3(1, 1, 1);
//uniform float u_matShininess; // = 64;

// Ouput data
out vec4 color;

void main()
{
	vec3 lightPosition_worldspace = vec3(-10, -10, 10);

	// Light emission properties
	// You probably want to put them as uniforms
	vec3 LightColor = vec3(1, 1, 1);
	float LightPower = 400.0f;
	
	// Material properties
	vec3 MaterialColor = fragmentColor.xyz;
	vec3 MaterialDiffuseColor = 0.5 * MaterialColor;
	vec3 MaterialAmbientColor = 0.5 * MaterialColor;
	vec3 MaterialSpecularColor = 0.2 * 0.3 * MaterialColor;

	// Distance to the light
	float distance = length( lightPosition_worldspace - position_worldspace );

	// Normal of the computed fragment, in camera space
	vec3 n = normalize( normal_cameraspace );
	// Direction of the light (from the fragment to the light)
	vec3 l = normalize( lightDirection_cameraspace );
	// Cosine of the angle between the normal and the light direction, 
	// clamped above 0
	//  - light is at the vertical of the triangle -> 1
	//  - light is perpendicular to the triangle -> 0
	//  - light is behind the triangle -> 0
	float cosTheta = clamp( dot( n, l ), 0,1 );
	
	// Eye vector (towards the camera)
	vec3 E = normalize(eyeDirection_cameraspace);
	// Direction in which the triangle reflects the light
	vec3 R = reflect(-l,n);
	// Cosine of the angle between the Eye vector and the Reflect vector,
	// clamped to 0
	//  - Looking into the reflection -> 1
	//  - Looking elsewhere -> < 1
	float cosAlpha = clamp( dot( E,R ), 0,1 );
	
	color.xyz =
		// Ambient : simulates indirect lighting
		MaterialAmbientColor +
		// Diffuse : "color" of the object
		MaterialDiffuseColor * LightColor * LightPower * cosTheta / (distance*distance) +
		// Specular : reflective highlight, like a mirror
		MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha,5) / (distance*distance);
	color.w = 1;

	// Output color = red 
	//color = fragmentColor;
}
