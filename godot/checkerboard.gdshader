shader_type spatial;

uniform vec3 color1 : source_color = vec3(0.8);
uniform vec3 color2 : source_color = vec3(0.2);

varying vec3 world_pos;

void vertex() {
    world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
    vec2 coord = world_pos.xz;
    
    vec2 grid = floor(coord);
    float checker = mod(grid.x + grid.y, 2.0);
    
    ALBEDO = mix(color1, color2, checker);
}