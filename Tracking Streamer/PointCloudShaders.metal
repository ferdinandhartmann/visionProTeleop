#include <metal_stdlib>
#include <RealityKit/RealityKit.h>

using namespace metal;

struct PointSpriteVertex {
    packed_half3 position;
    uchar4 color;
    half2 corner;
    ushort padding;
};

kernel void expandPointCloudSprites(
    device const uchar *packedPoints [[buffer(0)]],
    device PointSpriteVertex *vertices [[buffer(1)]],
    constant uint &pointCount [[buffer(2)]],
    uint pointIndex [[thread_position_in_grid]])
{
    if (pointIndex >= pointCount) {
        return;
    }

    const uint source = pointIndex * 9;
    const ushort xBits = ushort(packedPoints[source])
        | (ushort(packedPoints[source + 1]) << 8);
    const ushort yBits = ushort(packedPoints[source + 2])
        | (ushort(packedPoints[source + 3]) << 8);
    const ushort zBits = ushort(packedPoints[source + 4])
        | (ushort(packedPoints[source + 5]) << 8);
    const packed_half3 position = packed_half3(
        as_type<half>(xBits),
        as_type<half>(yBits),
        as_type<half>(zBits)
    );
    const uchar4 color = uchar4(
        packedPoints[source + 6],
        packedPoints[source + 7],
        packedPoints[source + 8],
        255
    );

    constexpr half rootThree = half(1.7320508);
    constexpr half2 corners[3] = {
        half2(-rootThree, -1.0h),
        half2( rootThree, -1.0h),
        half2(0.0h, 2.0h)
    };

    const uint destination = pointIndex * 3;
    for (uint vertex = 0; vertex < 3; ++vertex) {
        PointSpriteVertex output;
        output.position = position;
        output.color = color;
        output.corner = corners[vertex];
        output.padding = 0;
        vertices[destination + vertex] = output;
    }
}

[[visible]]
void pointCloudBillboard(realitykit::geometry_parameters params)
{
    const float radius = params.uniforms().custom_parameter().x;
    const float2 corner = params.geometry().uv0();
    const float4 viewOffset = float4(corner * radius, 0.0, 0.0);
    const float4x4 viewToModel = inverse(params.uniforms().model_to_view());
    params.geometry().set_model_position_offset((viewToModel * viewOffset).xyz);
}

static half3 srgbToLinear(half3 color)
{
    const half3 low = color / 12.92h;
    const half3 high = pow((color + 0.055h) / 1.055h, half3(2.4h));
    return select(low, high, color > 0.04045h);
}

[[visible]]
void pointCloudSurface(realitykit::surface_parameters params)
{
    if (length(params.geometry().uv0()) > 1.0) {
        discard_fragment();
    }
    const half3 color = half3(params.geometry().color().rgb);
    params.surface().set_emissive_color(srgbToLinear(color));
}
