import Foundation
import Metal
import RealityKit
import simd

@MainActor
final class PointCloudRenderer {

    // Keep the GPU allocation bounded. The Franka stream is decimated to well
    // below this value, while the wire decoder can still validate larger
    // protocol frames before this renderer rejects them.
    static let maximumPointCount = 20_000
    private static let verticesPerPoint = 12
    private static let indicesPerPoint = 60
    private static let colorTextureWidth = 512
    private static let colorTexturePointRows =
        (maximumPointCount + colorTextureWidth - 1) / colorTextureWidth
    private static let colorTextureHeight =
        colorTexturePointRows * 2

    let entity: ModelEntity

    private final class InputSlot {
        let buffer: MTLBuffer
        var busy = false

        init(buffer: MTLBuffer) {
            self.buffer = buffer
        }
    }

    private let commandQueue: MTLCommandQueue
    private let expansionPipeline: MTLComputePipelineState
    private let colorPipeline: MTLComputePipelineState
    private let lowLevelMesh: LowLevelMesh
    private let colorTexture: LowLevelTexture

    private var inputSlots: [InputSlot]
    private var pendingFrame: PointCloudFrame?

    private var spriteRadius: Float
    private var visible = true

    private var submittedFrames = 0
    private var gpuBusyDrops = 0
    private var lastStatsLog = 0.0

    init(spriteRadius: Float) throws {

        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue(),
              let library = device.makeDefaultLibrary(),
              let expansionFunction =
                library.makeFunction(name: "expandPointCloudSprites"),
              let colorFunction =
                library.makeFunction(name: "updatePointCloudColorTexture")
        else {
            throw PointCloudRendererError.metalUnavailable
        }

        self.commandQueue = commandQueue

        self.expansionPipeline =
            try device.makeComputePipelineState(
                function: expansionFunction
            )

        self.colorPipeline =
            try device.makeComputePipelineState(
                function: colorFunction
            )

        self.spriteRadius = spriteRadius

        var descriptor = LowLevelMesh.Descriptor()

        descriptor.vertexCapacity =
            Self.maximumPointCount * Self.verticesPerPoint

        descriptor.vertexAttributes = [
            .init(
                semantic: .position,
                format: .float3,
                layoutIndex: 0,
                offset: 0
            ),
            .init(
                semantic: .uv0,
                format: .float2,
                layoutIndex: 0,
                offset: 12
            ),
        ]

        descriptor.vertexLayouts = [
            .init(
                bufferIndex: 0,
                bufferStride: 20
            )
        ]

        descriptor.indexCapacity =
            Self.maximumPointCount * Self.indicesPerPoint

        descriptor.indexType = .uint32

        let lowLevelMesh =
            try LowLevelMesh(descriptor: descriptor)

        lowLevelMesh.withUnsafeMutableIndices { rawIndices in

            let indices =
                rawIndices.bindMemory(to: UInt32.self)

            let sphereIndices: [UInt32] = [
                0, 11, 5,  0, 5, 1,   0, 1, 7,   0, 7, 10,  0, 10, 11,
                1, 5, 9,   5, 11, 4,  11, 10, 2, 10, 7, 6,  7, 1, 8,
                3, 9, 4,   3, 4, 2,   3, 2, 6,   3, 6, 8,   3, 8, 9,
                4, 9, 5,   2, 4, 11,  6, 2, 10,  8, 6, 7,   9, 8, 1,
            ]
            for pointIndex in 0..<Self.maximumPointCount {
                let vertexBase = UInt32(
                    pointIndex * Self.verticesPerPoint
                )
                let indexBase = pointIndex * sphereIndices.count
                for (offset, localIndex) in sphereIndices.enumerated() {
                    indices[indexBase + offset] = vertexBase + localIndex
                }
            }
        }

        lowLevelMesh.parts.replaceAll([
            .init(
                indexCount: 0,
                topology: .triangle,
                materialIndex: 0,
                bounds: BoundingBox(
                    min: SIMD3<Float>(repeating: -0.001),
                    max: SIMD3<Float>(repeating: 0.001)
                )
            )
        ])

        self.lowLevelMesh = lowLevelMesh

        var textureDescriptor = LowLevelTexture.Descriptor()
        textureDescriptor.textureType = .type2D
        textureDescriptor.arrayLength = 1
        textureDescriptor.width = Self.colorTextureWidth
        textureDescriptor.height = Self.colorTextureHeight
        textureDescriptor.depth = 1
        textureDescriptor.mipmapLevelCount = 1
        textureDescriptor.pixelFormat = .bgra8Unorm
        textureDescriptor.textureUsage = [.shaderRead, .shaderWrite]
        textureDescriptor.swizzle = .init(
            red: .red,
            green: .green,
            blue: .blue,
            alpha: .alpha
        )

        let colorTexture =
            try LowLevelTexture(descriptor: textureDescriptor)
        self.colorTexture = colorTexture

        let entity = ModelEntity()

        entity.name = "pointCloudEntity"
        entity.isEnabled = false

        self.entity = entity

        let inputBytes =
            Self.maximumPointCount *
            PointCloudFrame.recordBytes

        self.inputSlots =
            try (0..<3).map { _ in

                guard let buffer =
                    device.makeBuffer(
                        length: inputBytes,
                        options: .storageModeShared
                    )
                else {
                    throw PointCloudRendererError
                        .bufferAllocationFailed
                }

                return InputSlot(buffer: buffer)
            }

        let meshResource =
            try MeshResource(from: lowLevelMesh)

        let textureResource =
            try TextureResource(from: colorTexture)

        let material =
            UnlitMaterial(texture: textureResource)

        entity.components.set(
            ModelComponent(
                mesh: meshResource,
                materials: [material]
            )
        )
    }

    func setSpriteRadius(_ radius: Float) {
        guard radius.isFinite,
              radius > 0,
              radius != spriteRadius
        else {
            return
        }

        spriteRadius = radius
    }

    func setVisible(_ isVisible: Bool) {
        visible = isVisible
        entity.isEnabled = isVisible && submittedFrames > 0
        if !isVisible {
            pendingFrame = nil
        }
    }

    func setPlacement(
        attachPosition: SIMD3<Float>?,
        attachRotation: simd_quatf?
    ) {
        let axisCorrection =
            simd_quatf(
                angle: -.pi / 2,
                axis: SIMD3<Float>(1, 0, 0)
            )

        if let attachPosition,
           let attachRotation {

            entity.transform.rotation =
                axisCorrection * attachRotation

            entity.transform.translation =
                axisCorrection.act(attachPosition)

        } else {

            entity.transform.rotation =
                axisCorrection

            entity.transform.translation =
                .zero
        }
    }
    
    func submit(_ frame: PointCloudFrame) {
    guard visible else {
        return
    }
    guard frame.pointCount > 0,
    frame.pointCount <= Self.maximumPointCount,
    frame.packedPoints.count ==
    frame.pointCount * PointCloudFrame.recordBytes
    else {
    return
    }

    guard let slot = inputSlots.first(where: { !$0.busy }) else {
        pendingFrame = frame
        gpuBusyDrops += 1
        return
    }

    pendingFrame = nil
    slot.busy = true

    frame.packedPoints.copyBytes(
        to: slot.buffer.contents()
            .assumingMemoryBound(to: UInt8.self),
        count: frame.packedPoints.count
    )

    guard let commandBuffer =
            commandQueue.makeCommandBuffer(),
          let encoder =
            commandBuffer.makeComputeCommandEncoder()
    else {
        slot.busy = false
        return
    }

    // LowLevelTexture requires the command buffer to be enqueued before
    // replace(using:) so RealityKit can synchronize the newly written texture
    // with its renderer. Without this, the material can sample an
    // uninitialized black resource and resource lifetime is undefined.
    commandBuffer.enqueue()

    let outputBuffer =
        lowLevelMesh.replace(
            bufferIndex: 0,
            using: commandBuffer
        )

    let outputColorTexture =
        colorTexture.replace(using: commandBuffer)

    var pointCount = UInt32(frame.pointCount)

    encoder.setComputePipelineState(
        colorPipeline
    )

    encoder.setBuffer(
        slot.buffer,
        offset: 0,
        index: 0
    )

    encoder.setBytes(
        &pointCount,
        length: MemoryLayout<UInt32>.size,
        index: 1
    )

    encoder.setTexture(
        outputColorTexture,
        index: 0
    )

    let colorThreadWidth =
        colorPipeline.threadExecutionWidth

    encoder.dispatchThreadgroups(
        MTLSize(
            width: (frame.pointCount + colorThreadWidth - 1) / colorThreadWidth,
            height: 1,
            depth: 1
        ),
        threadsPerThreadgroup: MTLSize(
            width: colorThreadWidth,
            height: 1,
            depth: 1
        )
    )

    encoder.setComputePipelineState(
        expansionPipeline
    )

    encoder.setBuffer(
        slot.buffer,
        offset: 0,
        index: 0
    )

    encoder.setBuffer(
        outputBuffer,
        offset: 0,
        index: 1
    )

    encoder.setBytes(
        &pointCount,
        length: MemoryLayout<UInt32>.size,
        index: 2
    )

    var radius = spriteRadius
    encoder.setBytes(
        &radius,
        length: MemoryLayout<Float>.size,
        index: 3
    )

    var colorTextureSize = SIMD2<UInt32>(
        UInt32(Self.colorTextureWidth),
        UInt32(Self.colorTextureHeight)
    )
    encoder.setBytes(
        &colorTextureSize,
        length: MemoryLayout<SIMD2<UInt32>>.size,
        index: 4
    )

    let threadWidth =
        expansionPipeline.threadExecutionWidth

    let threadgroupCount =
        MTLSize(
            width: (frame.pointCount + threadWidth - 1) / threadWidth,
            height: 1,
            depth: 1
        )

    let threadsPerGroup =
        MTLSize(
            width: threadWidth,
            height: 1,
            depth: 1
        )

    encoder.dispatchThreadgroups(
        threadgroupCount,
        threadsPerThreadgroup: threadsPerGroup
    )

    encoder.endEncoding()

    let margin =
        SIMD3<Float>(
            repeating: spriteRadius
        )

    let bounds = BoundingBox(
        min: frame.boundsMin - margin,
        max: frame.boundsMax + margin
    )

    lowLevelMesh.parts.replaceAll([
        .init(
            indexOffset: 0,
            indexCount: frame.pointCount * Self.indicesPerPoint,
            topology: .triangle,
            materialIndex: 0,
            bounds: bounds
        )
    ])

    commandBuffer.addCompletedHandler {
        [weak self, weak slot] completedBuffer in

        Task { @MainActor in
            guard let self,
                  let slot
            else {
                return
            }

            slot.busy = false

            if let error = completedBuffer.error {
                dlog(
                    "❌ [PointCloud GPU] command buffer failed: " +
                    error.localizedDescription
                )
            }

            if let pending = self.pendingFrame {
                self.pendingFrame = nil
                self.submit(pending)
            }
        }
    }

    commandBuffer.commit()

    if !entity.isEnabled {
        entity.isEnabled = visible
    }

    submittedFrames += 1

    logStatsIfNeeded(
        frame: frame
    )

    }


    func reset() {
        pendingFrame = nil
        entity.isEnabled = false
    }

    private func logStatsIfNeeded(
        frame: PointCloudFrame
    ) {
        let now =
            ProcessInfo.processInfo.systemUptime

        guard now - lastStatsLog >= 1.0 else {
            return
        }

        lastStatsLog = now

        let colorSummary = sampledColorSummary(frame)

        dlog(
            "🌫️ [PointCloud GPU] " +
            "points=\(frame.pointCount) " +
            "submitted=\(submittedFrames) " +
            "gpu_busy_drops=\(gpuBusyDrops) " +
            colorSummary
        )
    }

    private func sampledColorSummary(
        _ frame: PointCloudFrame
    ) -> String {
        let targetSamples = min(frame.pointCount, 256)
        guard targetSamples > 0 else {
            return "rgb_samples=0"
        }

        let pointStride = max(1, frame.pointCount / targetSamples)
        var minimum = UInt8.max
        var maximum = UInt8.min
        var redTotal: UInt64 = 0
        var greenTotal: UInt64 = 0
        var blueTotal: UInt64 = 0
        var sampleCount: UInt64 = 0

        frame.packedPoints.withUnsafeBytes { rawBuffer in
            let bytes = rawBuffer.bindMemory(to: UInt8.self)
            var pointIndex = 0
            while pointIndex < frame.pointCount,
                  sampleCount < UInt64(targetSamples) {
                let colorOffset =
                    pointIndex * PointCloudFrame.recordBytes + 6
                let red = bytes[colorOffset]
                let green = bytes[colorOffset + 1]
                let blue = bytes[colorOffset + 2]
                minimum = Swift.min(
                    minimum,
                    Swift.min(red, Swift.min(green, blue))
                )
                maximum = Swift.max(
                    maximum,
                    Swift.max(red, Swift.max(green, blue))
                )
                redTotal += UInt64(red)
                greenTotal += UInt64(green)
                blueTotal += UInt64(blue)
                sampleCount += 1
                pointIndex += pointStride
            }
        }

        guard sampleCount > 0 else {
            return "rgb_samples=0"
        }
        return
            "rgb_avg=(\(redTotal / sampleCount)," +
            "\(greenTotal / sampleCount)," +
            "\(blueTotal / sampleCount)) " +
            "rgb_range=\(minimum)...\(maximum)"
    }
}

enum PointCloudRendererError: Error {
    case metalUnavailable
    case bufferAllocationFailed
}
