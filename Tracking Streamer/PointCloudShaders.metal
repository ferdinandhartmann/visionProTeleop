#include <metal_stdlib>

using namespace metal;

// This layout intentionally matches PointCloudRenderer's RealityKit descriptor:
// packed_float3 at byte 0 followed by uchar4 at byte 12 (16-byte stride).
struct PointCloudVertex {
    packed_float3 position;
    uchar4 color;
};

kernel void expandPointCloudSprites(
    device const uchar *packedPoints [[buffer(0)]],
    device PointCloudVertex *vertices [[buffer(1)]],
    constant uint &pointCount [[buffer(2)]],
    constant float &radius [[buffer(3)]],
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
    const uchar4 color = uchar4(
        packedPoints[sourceOffset + 6u],
        packedPoints[sourceOffset + 7u],
        packedPoints[sourceOffset + 8u],
        255
    );

    // A tiny tetrahedron is visible from every viewing direction and needs no
    // camera-facing geometry modifier. The old implementation emitted three
    // identical positions, producing a zero-area triangle.
    constexpr float invRootThree = 0.57735026919f;
    const float3 offsets[4] = {
        float3( 1.0f,  1.0f,  1.0f) * invRootThree,
        float3( 1.0f, -1.0f, -1.0f) * invRootThree,
        float3(-1.0f,  1.0f, -1.0f) * invRootThree,
        float3(-1.0f, -1.0f,  1.0f) * invRootThree
    };

    const uint destination = pointIndex * 4u;
    for (uint vertexIndex = 0u; vertexIndex < 4u; ++vertexIndex) {
        vertices[destination + vertexIndex] = PointCloudVertex{
            packed_float3(center + offsets[vertexIndex] * radius), color
        };
    }
}
