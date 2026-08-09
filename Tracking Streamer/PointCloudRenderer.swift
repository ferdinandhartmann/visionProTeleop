import Foundation
import Metal
import RealityKit
import simd

@MainActor
final class PointCloudRenderer {

    static let maximumPointCount = PointCloudFrame.maximumPointCount

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
    private let lowLevelMesh: LowLevelMesh

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
                library.makeFunction(name: "expandPointCloudSprites")
        else {
            throw PointCloudRendererError.metalUnavailable
        }

        self.commandQueue = commandQueue

        self.expansionPipeline =
            try device.makeComputePipelineState(
                function: expansionFunction
            )

        self.spriteRadius = spriteRadius

        var descriptor = LowLevelMesh.Descriptor()

        descriptor.vertexCapacity =
            Self.maximumPointCount * 4

        descriptor.vertexAttributes = [
            .init(
                semantic: .position,
                format: .float3,
                layoutIndex: 0,
                offset: 0
            ),
            .init(
                semantic: .color,
                format: .uchar4Normalized,
                layoutIndex: 0,
                offset: 12
            ),
        ]

        descriptor.vertexLayouts = [
            .init(
                bufferIndex: 0,
                bufferStride: 16
            )
        ]

        descriptor.indexCapacity =
            Self.maximumPointCount * 12

        descriptor.indexType = .uint32

        let lowLevelMesh =
            try LowLevelMesh(descriptor: descriptor)

        lowLevelMesh.withUnsafeMutableIndices { rawIndices in

            let indices =
                rawIndices.bindMemory(to: UInt32.self)

            let tetrahedron: [UInt32] = [
                0, 1, 2,
                0, 3, 1,
                0, 2, 3,
                1, 3, 2,
            ]
            for pointIndex in 0..<Self.maximumPointCount {
                let vertexBase = UInt32(pointIndex * 4)
                let indexBase = pointIndex * tetrahedron.count
                for (offset, localIndex) in tetrahedron.enumerated() {
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

        let material =
            UnlitMaterial(color: .white)

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

    let outputBuffer =
        lowLevelMesh.replace(
            bufferIndex: 0,
            using: commandBuffer
        )

    var pointCount = UInt32(frame.pointCount)

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
            indexCount: frame.pointCount * 12,
            topology: .triangle,
            materialIndex: 0,
            bounds: bounds
        )
    ])

    commandBuffer.addCompletedHandler {
        [weak self, weak slot] _ in

        Task { @MainActor in
            guard let self,
                  let slot
            else {
                return
            }

            slot.busy = false

            if let pending = self.pendingFrame {
                self.pendingFrame = nil
                self.submit(pending)
            }
        }
    }

    commandBuffer.commit()

    entity.isEnabled = visible

    submittedFrames += 1

    logStatsIfNeeded(
        pointCount: frame.pointCount
    )

    }


    func reset() {
        pendingFrame = nil
        entity.isEnabled = false
    }

    private func logStatsIfNeeded(
        pointCount: Int
    ) {
        let now =
            ProcessInfo.processInfo.systemUptime

        guard now - lastStatsLog >= 1.0 else {
            return
        }

        lastStatsLog = now

        dlog(
            "🌫️ [PointCloud GPU] " +
            "points=\(pointCount) " +
            "submitted=\(submittedFrames) " +
            "gpu_busy_drops=\(gpuBusyDrops)"
        )
    }
}

enum PointCloudRendererError: Error {
    case metalUnavailable
    case bufferAllocationFailed
}
