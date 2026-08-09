#include <metal_stdlib>

using namespace metal;

// This layout intentionally matches PointCloudRenderer's RealityKit descriptor:
// packed_float3 at byte 0 followed by packed_float2 at byte 12 (20-byte stride).
struct PointCloudVertex {
    packed_float3 position;
    packed_float2 uv;
};

// Copy each wire RGB triplet into a texel. A standard UnlitMaterial samples
// this dynamic texture, avoiding RealityKit's unavailable legacy Metal header
// and its failure to automatically consume LowLevelMesh vertex colors.
kernel void updatePointCloudColorTexture(
    device const uchar *packedPoints [[buffer(0)]],
    texture2d<half, access::write> colorTexture [[texture(0)]],
    constant uint &pointCount [[buffer(1)]],
    uint pointIndex [[thread_position_in_grid]])
{
    if (pointIndex >= pointCount) {
        return;
    }

    const uint sourceOffset = pointIndex * 9u;
    const half inverseByte = half(1.0h / 255.0h);
    const half4 color = half4(
        half(packedPoints[sourceOffset + 6u]) * inverseByte,
        half(packedPoints[sourceOffset + 7u]) * inverseByte,
        half(packedPoints[sourceOffset + 8u]) * inverseByte,
        1.0h
    );
    const uint2 texel(
        pointIndex % colorTexture.get_width(),
        pointIndex / colorTexture.get_width()
    );
    colorTexture.write(color, texel);

    // RealityKit material UV conventions can differ between procedural and
    // imported geometry. Keep a mirrored copy in a disjoint half of the
    // texture so either V-axis convention samples the same camera RGB value.
    const uint2 mirroredTexel(
        texel.x,
        colorTexture.get_height() - 1u - texel.y
    );
    colorTexture.write(color, mirroredTexel);
}

kernel void expandPointCloudSprites(
    device const uchar *packedPoints [[buffer(0)]],
    device PointCloudVertex *vertices [[buffer(1)]],
    constant uint &pointCount [[buffer(2)]],
    constant float &radius [[buffer(3)]],
    constant uint2 &colorTextureSize [[buffer(4)]],
    uint pointIndex [[thread_position_in_grid]])
{
    if (pointIndex >= pointCount) {
        return;
    }

    // Wire record: little-endian float16 XYZ followed by RGB (9 bytes).
    const uint sourceOffset = pointIndex * 9u;
    const ushort xBits = ushort(packedPoints[sourceOffset]) |
        (ushort(packedPoints[sourceOffset + 1u]) << 8);
    const ushort yBits = ushort(packedPoints[sourceOffset + 2u]) |
        (ushort(packedPoints[sourceOffset + 3u]) << 8);
    const ushort zBits = ushort(packedPoints[sourceOffset + 4u]) |
        (ushort(packedPoints[sourceOffset + 5u]) << 8);

    // Reinterpret the IEEE-754 bit pattern; half(xBits) would numerically
    // convert the integer and corrupt every point position.
    const float3 center = float3(
        float(as_type<half>(xBits)),
        float(as_type<half>(yBits)),
        float(as_type<half>(zBits))
    );
    const uint2 colorTexel(
        pointIndex % colorTextureSize.x,
        pointIndex / colorTextureSize.x
    );
    const float2 uv = (
        float2(colorTexel) + float2(0.5f)
    ) / float2(colorTextureSize);

    // An icosahedron gives every point a round, sphere-like silhouette from
    // all view directions while retaining one exact source RGB value.
    constexpr float phi = 1.61803398875f;
    constexpr float normalization = 0.52573111212f;
    const float3 offsets[12] = {
        float3(-1.0f,  phi,  0.0f) * normalization,
        float3( 1.0f,  phi,  0.0f) * normalization,
        float3(-1.0f, -phi,  0.0f) * normalization,
        float3( 1.0f, -phi,  0.0f) * normalization,
        float3( 0.0f, -1.0f,  phi) * normalization,
        float3( 0.0f,  1.0f,  phi) * normalization,
        float3( 0.0f, -1.0f, -phi) * normalization,
        float3( 0.0f,  1.0f, -phi) * normalization,
        float3( phi,   0.0f, -1.0f) * normalization,
        float3( phi,   0.0f,  1.0f) * normalization,
        float3(-phi,   0.0f, -1.0f) * normalization,
        float3(-phi,   0.0f,  1.0f) * normalization
    };

    const uint destination = pointIndex * 12u;
    for (uint vertexIndex = 0u; vertexIndex < 12u; ++vertexIndex) {
        vertices[destination + vertexIndex] = PointCloudVertex{
            packed_float3(center + offsets[vertexIndex] * radius),
            packed_float2(uv)
        };
    }
}
