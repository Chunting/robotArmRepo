#define PI (3.1415926536)
#define TWO_PI (6.2831853072)

#ifdef GL_ES
precision mediump float;
#endif

uniform float elapsedTime;
uniform float stage;
uniform float alpha;
varying vec3 position, normal;
varying float randomOffset;

//float time = elapsedTime;

const vec4 on = vec4(1.);
const vec4 g  = vec4(0,1,0,1);
const vec4 off = vec4(vec3(0.), 1.);

void main() {
//	float stage = 4.;
	
	if(stage < 1.) {
        vec3 col = vec3(1.0, 0., 1.0)*dot(normal,vec3(1.0, 0.0, 1.0))+vec3(1.0, 0., 1.0);
        gl_FragColor = vec4(col, 1.);
	} else if(stage < 2.) {
        vec3 col = vec3(1.0, 1.0, 0.0)*dot(normal,vec3(1.0, 1.0, 0.0))+vec3(1.0, 1.0, 0.0);
        gl_FragColor = vec4(col, 1.);
	} else if(stage < 3.) {
        vec3 col = vec3(0.0, 1.0, 1.0)*dot(normal,vec3(0.0, 1.0, 1.0))+vec3(0.0, 0.1, 1.0);
        gl_FragColor = vec4(col, 1.);
    } else if(stage < 4.) {
        vec3 col = vec3(1.0)*dot(normal,vec3(1.0))+vec3(1.0);
        gl_FragColor = vec4(col, 1.);
    }
}
