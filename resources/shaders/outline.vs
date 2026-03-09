#version 330

in vec3 vertexPosition;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform float outlineWidth;
uniform vec2 viewportSize;
uniform vec3 objectCenter;  // model-local space center of the mesh

void main()
{
    vec4 clipPos = mvp * vec4(vertexPosition, 1.0);
    vec4 clipCenter = mvp * vec4(objectCenter, 1.0);

    // Direction from object center to vertex in NDC
    vec2 ndcPos = clipPos.xy / clipPos.w;
    vec2 ndcCenter = clipCenter.xy / clipCenter.w;
    vec2 dir = ndcPos - ndcCenter;
    float len = length(dir);
    if (len > 0.001) {
        dir /= len;
        // Offset by outlineWidth pixels in screen space
        // (2.0 / viewportSize) converts pixels to NDC; * clipPos.w undoes perspective divide
        clipPos.xy += dir * outlineWidth * (2.0 / viewportSize) * clipPos.w;
    }

    gl_Position = clipPos;
}
