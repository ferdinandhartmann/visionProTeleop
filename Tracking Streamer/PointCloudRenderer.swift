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
    private var material: CustomMaterial
    private var inputSlots: [InputSlot]
    private var pendingFrame: PointCloudFrame?
    private var spriteRadius: Float
    private var submittedFrames = 0
    private var gpuBusyDrops = 0
    private var lastStatsLog = 0.0

    init(spriteRadius: Float) async throws {
        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue(),
              let library = device.makeDefaultLibrary(),
              let expansionFunction = library.makeFunction(name: "expandPointCloudSprites") else {
            throw PointCloudRendererError.metalUnavailable
        }

        self.commandQueue = commandQueue
        self.expansionPipeline = try device.makeComputePipelineState(
            function: expansionFunction
        )
        self.spriteRadius = spriteRadius

        var descriptor = LowLevelMesh.Descriptor()
        descriptor.vertexCapacity = Self.maximumPointCount * 3
        descriptor.vertexAttributes = [
            .init(semantic: .position, format: .half3, layoutIndex: 0, offset: 0),
            .init(
                semantic: .color,
                format: .uchar4Normalized,
                layoutIndex: 0,
                offset: 6
            ),
            .init(semantic: .uv0, format: .half2, layoutIndex: 0, offset: 10),
        ]
        descriptor.vertexLayouts = [
            .init(bufferIndex: 0, bufferStride: 16)
        ]
        descriptor.indexCapacity = Self.maximumPointCount * 3
        descriptor.indexType = .uint32

        let lowLevelMesh = try LowLevelMesh(descriptor: descriptor)
        lowLevelMesh.withUnsafeMutableIndices { rawIndices in
            let indices = rawIndices.bindMemory(to: UInt32.self)
            for index in 0..<indices.count {
                indices[index] = UInt32(index)
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

        let surfaceShader = CustomMaterial.SurfaceShader(
            named: "pointCloudSurface",
            in: library
        )
        let geometryModifier = CustomMaterial.GeometryModifier(
            named: "pointCloudBillboard",
            in: library
        )
        var material = try CustomMaterial(
            surfaceShader: surfaceShader,
            geometryModifier: geometryModifier,
            lightingModel: .unlit
        )
        material.custom.value = SIMD4<Float>(spriteRadius, 0, 0, 0)
        material.faceCulling = .none
        self.material = material

        let entity = ModelEntity()
        entity.name = "pointCloudEntity"
        entity.isEnabled = false
        self.entity = entity

        let inputBytes = Self.maximumPointCount * PointCloudFrame.recordBytes
        self.inputSlots = try (0..<3).map { _ in
            guard let buffer = device.makeBuffer(
                length: inputBytes,
                options: .storageModeShared
            ) else {
                throw PointCloudRendererError.bufferAllocationFailed
            }
            return InputSlot(buffer: buffer)
        }

        let meshResource = try await MeshResource(from: lowLevelMesh)
        entity.components.set(
            ModelComponent(mesh: meshResource, materials: [material])
        )
    }

    func setSpriteRadius(_ radius: Float) {
        guard radius.isFinite, radius > 0, radius != spriteRadius else { return }
        spriteRadius = radius
        material.custom.value = SIMD4<Float>(radius, 0, 0, 0)
        entity.model?.materials = [material]
    }

    func setPlacement(
        attachPosition: SIMD3<Float>?,
        attachRotation: simd_quatf?
    ) {
        let axisCorrection = simd_quatf(
            angle: -.pi / 2,
            axis: SIMD3<Float>(1, 0, 0)
        )
        if let attachPosition, let attachRotation {
            entity.transform.rotation = axisCorrection * attachRotation
            entity.transform.translation = axisCorrection.act(attachPosition)
        } else {
            entity.transform.rotation = axisCorrection
            entity.transform.translation = .zero
        }
    }

    func submit(_ frame: PointCloudFrame) {
        guard frame.pointCount > 0,
              frame.pointCount <= Self.maximumPointCount,
              frame.packedPoints.count == frame.pointCount * PointCloudFrame.recordBytes else {
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
            to: slot.buffer.contents().assumingMemoryBound(to: UInt8.self),
            count: frame.packedPoints.count
        )

        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder() else {
            slot.busy = false
            return
        }

        let outputBuffer = lowLevelMesh.replace(bufferIndex: 0, using: commandBuffer)
        var pointCount = UInt32(frame.pointCount)
        encoder.setComputePipelineState(expansionPipeline)
        encoder.setBuffer(slot.buffer, offset: 0, index: 0)
        encoder.setBuffer(outputBuffer, offset: 0, index: 1)
        encoder.setBytes(
            &pointCount,
            length: MemoryLayout<UInt32>.size,
            index: 2
        )
        let width = expansionPipeline.threadExecutionWidth
        encoder.dispatchThreads(
            MTLSize(width: frame.pointCount, height: 1, depth: 1),
            threadsPerThreadgroup: MTLSize(width: width, height: 1, depth: 1)
        )
        encoder.endEncoding()

        let margin = SIMD3<Float>(repeating: spriteRadius)
        let bounds = BoundingBox(
            min: frame.boundsMin - margin,
            max: frame.boundsMax + margin
        )
        lowLevelMesh.parts.replaceAll([
            .init(
                indexOffset: 0,
                indexCount: frame.pointCount * 3,
                topology: .triangle,
                materialIndex: 0,
                bounds: bounds
            )
        ])

        commandBuffer.addCompletedHandler { [weak self, weak slot] _ in
            Task { @MainActor in
                guard let self, let slot else { return }
                slot.busy = false
                if let pending = self.pendingFrame {
                    self.pendingFrame = nil
                    self.submit(pending)
                }
            }
        }
        commandBuffer.commit()

        entity.isEnabled = true
        submittedFrames += 1
        logStatsIfNeeded(pointCount: frame.pointCount)
    }

    func reset() {
        pendingFrame = nil
        entity.isEnabled = false
    }

    private func logStatsIfNeeded(pointCount: Int) {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastStatsLog >= 1.0 else { return }
        lastStatsLog = now
        dlog(
            "🌫️ [PointCloud GPU] points=\(pointCount) submitted=\(submittedFrames) " +
            "gpu_busy_drops=\(gpuBusyDrops)"
        )
    }
}

enum PointCloudRendererError: Error {
    case metalUnavailable
    case bufferAllocationFailed
}
