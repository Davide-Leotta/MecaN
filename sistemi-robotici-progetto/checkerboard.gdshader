shader_type spatial;

uniform float scale : hint_range(1.0, 100.0) = 100.0;
uniform vec3 color1 : source_color = vec3(0.8);
uniform vec3 color2 : source_color = vec3(0.2);

void fragment() {
    vec2 uv = floor(UV * scale);
    float checker = mod(uv.x + uv.y, 2.0);
    
    ALBEDO = mix(color1, color2, checker);
}