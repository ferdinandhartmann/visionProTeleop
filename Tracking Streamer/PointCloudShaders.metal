#include <metal_stdlib>

using namespace metal;

// MARK: - Point Sprite Vertex

struct PointSpriteVertex {
half3  position;
uchar4 color;
half2  corner;
ushort padding;
};

// MARK: - Expand Point Cloud

kernel void expandPointCloudSprites(
device const uchar *packedPoints [[buffer(0)]],
device PointSpriteVertex *vertices [[buffer(1)]],
constant uint &pointCount [[buffer(2)]],
uint pointIndex [[thread_position_in_grid]]
)
{
if (pointIndex >= pointCount) {
return;
}


// One point is 9 bytes:
//
// X:   2 bytes
// Y:   2 bytes
// Z:   2 bytes
// RGB: 3 bytes

const uint sourceOffset = pointIndex * 9u;

const ushort xBits =
    ushort(packedPoints[sourceOffset]) |
    (ushort(packedPoints[sourceOffset + 1u]) << 8);

const ushort yBits =
    ushort(packedPoints[sourceOffset + 2u]) |
    (ushort(packedPoints[sourceOffset + 3u]) << 8);

const ushort zBits =
    ushort(packedPoints[sourceOffset + 4u]) |
    (ushort(packedPoints[sourceOffset + 5u]) << 8);

const half3 position = half3(
    half(xBits),
    half(yBits),
    half(zBits)
);

const uchar4 color = uchar4(
    packedPoints[sourceOffset + 6u],
    packedPoints[sourceOffset + 7u],
    packedPoints[sourceOffset + 8u],
    255
);

constexpr half rootThree = 1.7320508h;

constexpr half2 corners[3] = {
    half2(-rootThree, -1.0h),
    half2( rootThree, -1.0h),
    half2( 0.0h,        2.0h)
};

const uint destination = pointIndex * 3u;

vertices[destination + 0u] = PointSpriteVertex{
    position,
    color,
    corners[0],
    0
};

vertices[destination + 1u] = PointSpriteVertex{
    position,
    color,
    corners[1],
    0
};

vertices[destination + 2u] = PointSpriteVertex{
    position,
    color,
    corners[2],
    0
};


}
