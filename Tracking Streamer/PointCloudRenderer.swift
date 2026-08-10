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
    private static let verticesPerPoint = 9
    private static let indicesPerPoint = 24
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

    private struct PendingSubmission {
        let frame: PointCloudFrame
        let billboardRight: SIMD3<Float>
        let billboardUp: SIMD3<Float>
    }

    private struct AdaptiveDensityController {
        private(set) var sourceStride = 1
        private(set) var latencySamples: [Double] = []
        private var slowFrameStreak = 0
        private var busyDropStreak = 0
        private var lastBusyDropAt: Double?
        private var stableSince: Double?
        private var lastTransition = -Double.infinity

        var modeName: String {
            switch sourceStride {
            case 1: return "full"
            case 2: return "half"
            default: return "quarter"
            }
        }

        var averageLatencyMilliseconds: Double {
            guard !latencySamples.isEmpty else { return 0 }
            return latencySamples.reduce(0, +) /
                Double(latencySamples.count) * 1_000
        }

        var p95LatencyMilliseconds: Double {
            guard !latencySamples.isEmpty else { return 0 }
            let sorted = latencySamples.sorted()
            let index = min(
                sorted.count - 1,
                Int((Double(sorted.count) * 0.95).rounded(.up)) - 1
            )
            return sorted[max(0, index)] * 1_000
        }

        mutating func recordCompletion(
            latency: Double,
            gpuDuration: Double?,
            thermalState: ProcessInfo.ThermalState,
            now: Double
        ) -> Bool {
            let measured = max(latency, gpuDuration ?? 0)
            latencySamples.append(measured)
            if latencySamples.count > 120 {
                latencySamples.removeFirst(latencySamples.count - 120)
            }

            if applyThermalMinimum(thermalState, now: now) {
                return true
            }

            if measured > 0.012 {
                slowFrameStreak += 1
                stableSince = nil
            } else {
                slowFrameStreak = 0
                if measured < 0.006 {
                    stableSince = stableSince ?? now
                } else {
                    stableSince = nil
                }
            }

            if slowFrameStreak >= 10,
               now - lastTransition >= 3.0,
               sourceStride < 4 {
                sourceStride *= 2
                didTransition(at: now)
                return true
            }

            if let stableSince,
               now - stableSince >= 5.0,
               now - lastTransition >= 3.0,
               sourceStride > thermalMinimumStride(thermalState) {
                sourceStride /= 2
                didTransition(at: now)
                return true
            }

            return false
        }

        mutating func recordBusyDrop(
            thermalState: ProcessInfo.ThermalState,
            now: Double
        ) -> Bool {
            stableSince = nil
            slowFrameStreak = 0
            if let lastBusyDropAt, now - lastBusyDropAt <= 1.0 {
                busyDropStreak += 1
            } else {
                busyDropStreak = 1
            }
            lastBusyDropAt = now

            if applyThermalMinimum(thermalState, now: now) {
                return true
            }

            if busyDropStreak >= 2,
               now - lastTransition >= 3.0,
               sourceStride < 4 {
                sourceStride *= 2
                didTransition(at: now)
                return true
            }
            return false
        }

        mutating func reset() {
            self = AdaptiveDensityController()
        }

        private func thermalMinimumStride(
            _ thermalState: ProcessInfo.ThermalState
        ) -> Int {
            switch thermalState {
            case .critical: return 4
            case .serious: return 2
            default: return 1
            }
        }

        private mutating func applyThermalMinimum(
            _ thermalState: ProcessInfo.ThermalState,
            now: Double
        ) -> Bool {
            let minimum = thermalMinimumStride(thermalState)
            guard sourceStride < minimum else { return false }
            sourceStride = minimum
            didTransition(at: now)
            return true
        }

        private mutating func didTransition(at now: Double) {
            lastTransition = now
            slowFrameStreak = 0
            busyDropStreak = 0
            lastBusyDropAt = nil
            stableSince = nil
        }
    }

    private let commandQueue: MTLCommandQueue
    private let expansionPipeline: MTLComputePipelineState
    private let colorPipeline: MTLComputePipelineState
    private let lowLevelMesh: LowLevelMesh
    private let colorTexture: LowLevelTexture

    private var inputSlots: [InputSlot]
    private var pendingSubmission: PendingSubmission?
    private var adaptiveDensity = AdaptiveDensityController()

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

            for pointIndex in 0..<Self.maximumPointCount {
                let vertexBase = UInt32(
                    pointIndex * Self.verticesPerPoint
                )
                let indexBase = pointIndex * Self.indicesPerPoint
                for segment in 0..<8 {
                    let segmentBase = indexBase + segment * 3
                    indices[segmentBase] = vertexBase
                    indices[segmentBase + 1] =
                        vertexBase + UInt32(segment + 1)
                    indices[segmentBase + 2] =
                        vertexBase + UInt32((segment + 1) % 8 + 1)
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
            pendingSubmission = nil
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
    
    func submit(
        _ frame: PointCloudFrame,
        headWorldTransform: simd_float4x4?
    ) {
        let axes = billboardAxes(
            headWorldTransform: headWorldTransform
        )
        submit(
            PendingSubmission(
                frame: frame,
                billboardRight: axes.right,
                billboardUp: axes.up
            )
        )
    }

    private func submit(_ submission: PendingSubmission) {
        let frame = submission.frame
        guard visible else { return }
        guard frame.pointCount > 0,
              frame.pointCount <= Self.maximumPointCount,
              frame.packedPoints.count ==
                frame.pointCount * PointCloudFrame.recordBytes
        else {
            return
        }

        guard let slot = inputSlots.first(where: { !$0.busy }) else {
            pendingSubmission = submission
            gpuBusyDrops += 1
            let changed = adaptiveDensity.recordBusyDrop(
                thermalState: ProcessInfo.processInfo.thermalState,
                now: ProcessInfo.processInfo.systemUptime
            )
            if changed {
                logAdaptiveTransition()
            }
            return
        }

        pendingSubmission = nil
        slot.busy = true

        frame.packedPoints.copyBytes(
            to: slot.buffer.contents()
                .assumingMemoryBound(to: UInt8.self),
            count: frame.packedPoints.count
        )

        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder()
        else {
            slot.busy = false
            return
        }

        let sourceStride = adaptiveDensity.sourceStride
        let renderedPointCount =
            (frame.pointCount + sourceStride - 1) / sourceStride
        var renderedPointCountGPU = UInt32(renderedPointCount)
        var sourceStrideGPU = UInt32(sourceStride)

        // LowLevelTexture requires the command buffer to be enqueued before
        // replace(using:) so RealityKit can synchronize the updated resources.
        commandBuffer.enqueue()

        let outputBuffer = lowLevelMesh.replace(
            bufferIndex: 0,
            using: commandBuffer
        )
        let outputColorTexture = colorTexture.replace(using: commandBuffer)

        encoder.setComputePipelineState(colorPipeline)
        encoder.setBuffer(slot.buffer, offset: 0, index: 0)
        encoder.setBytes(
            &renderedPointCountGPU,
            length: MemoryLayout<UInt32>.size,
            index: 1
        )
        encoder.setBytes(
            &sourceStrideGPU,
            length: MemoryLayout<UInt32>.size,
            index: 2
        )
        encoder.setTexture(outputColorTexture, index: 0)

        let colorThreadWidth = colorPipeline.threadExecutionWidth
        encoder.dispatchThreadgroups(
            MTLSize(
                width: (renderedPointCount + colorThreadWidth - 1) /
                    colorThreadWidth,
                height: 1,
                depth: 1
            ),
            threadsPerThreadgroup: MTLSize(
                width: colorThreadWidth,
                height: 1,
                depth: 1
            )
        )

        encoder.setComputePipelineState(expansionPipeline)
        encoder.setBuffer(slot.buffer, offset: 0, index: 0)
        encoder.setBuffer(outputBuffer, offset: 0, index: 1)
        encoder.setBytes(
            &renderedPointCountGPU,
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

        var billboardRight = submission.billboardRight
        var billboardUp = submission.billboardUp
        encoder.setBytes(
            &billboardRight,
            length: MemoryLayout<SIMD3<Float>>.stride,
            index: 5
        )
        encoder.setBytes(
            &billboardUp,
            length: MemoryLayout<SIMD3<Float>>.stride,
            index: 6
        )
        encoder.setBytes(
            &sourceStrideGPU,
            length: MemoryLayout<UInt32>.size,
            index: 7
        )

        let threadWidth = expansionPipeline.threadExecutionWidth
        encoder.dispatchThreadgroups(
            MTLSize(
                width: (renderedPointCount + threadWidth - 1) /
                    threadWidth,
                height: 1,
                depth: 1
            ),
            threadsPerThreadgroup: MTLSize(
                width: threadWidth,
                height: 1,
                depth: 1
            )
        )
        encoder.endEncoding()

        let margin = SIMD3<Float>(repeating: spriteRadius)
        lowLevelMesh.parts.replaceAll([
            .init(
                indexOffset: 0,
                indexCount: renderedPointCount * Self.indicesPerPoint,
                topology: .triangle,
                materialIndex: 0,
                bounds: BoundingBox(
                    min: frame.boundsMin - margin,
                    max: frame.boundsMax + margin
                )
            )
        ])

        let submittedAt = ProcessInfo.processInfo.systemUptime
        commandBuffer.addCompletedHandler {
            [weak self, weak slot] completedBuffer in
            let completedAt = ProcessInfo.processInfo.systemUptime
            let gpuDuration: Double? =
                completedBuffer.gpuEndTime > completedBuffer.gpuStartTime
                ? completedBuffer.gpuEndTime - completedBuffer.gpuStartTime
                : nil

            Task { @MainActor in
                guard let self, let slot else { return }
                slot.busy = false

                if let error = completedBuffer.error {
                    dlog(
                        "❌ [PointCloud GPU] command buffer failed: " +
                        error.localizedDescription
                    )
                }

                let changed = self.adaptiveDensity.recordCompletion(
                    latency: completedAt - submittedAt,
                    gpuDuration: gpuDuration,
                    thermalState: ProcessInfo.processInfo.thermalState,
                    now: completedAt
                )
                if changed {
                    self.logAdaptiveTransition()
                }

                if let pending = self.pendingSubmission {
                    self.pendingSubmission = nil
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
            frame: frame,
            renderedPointCount: renderedPointCount
        )
    }

    private func billboardAxes(
        headWorldTransform: simd_float4x4?
    ) -> (right: SIMD3<Float>, up: SIMD3<Float>) {
        let fallback = (
            right: SIMD3<Float>(1, 0, 0),
            up: SIMD3<Float>(0, 1, 0)
        )
        guard let headWorldTransform,
              matrixIsFinite(headWorldTransform)
        else {
            return fallback
        }

        let worldFromLocal = entity.transformMatrix(relativeTo: nil)
        guard matrixIsFinite(worldFromLocal),
              abs(simd_determinant(worldFromLocal)) > 1e-8
        else {
            return fallback
        }

        let localFromWorld = simd_inverse(worldFromLocal)
        let headRightWorld = headWorldTransform.columns.0
        let headUpWorld = headWorldTransform.columns.1
        let localRight4 = localFromWorld * SIMD4<Float>(
            headRightWorld.x,
            headRightWorld.y,
            headRightWorld.z,
            0
        )
        let localUp4 = localFromWorld * SIMD4<Float>(
            headUpWorld.x,
            headUpWorld.y,
            headUpWorld.z,
            0
        )
        var right = SIMD3<Float>(
            localRight4.x,
            localRight4.y,
            localRight4.z
        )
        var up = SIMD3<Float>(localUp4.x, localUp4.y, localUp4.z)

        let rightLength = simd_length(right)
        guard rightLength.isFinite, rightLength > 1e-6 else {
            return fallback
        }
        right /= rightLength

        // Remove any scale/shear-induced component parallel to right so the
        // splat remains circular after placement transforms.
        up -= right * simd_dot(up, right)
        let upLength = simd_length(up)
        guard upLength.isFinite, upLength > 1e-6 else {
            return fallback
        }
        up /= upLength
        return (right, up)
    }

    private func matrixIsFinite(_ matrix: simd_float4x4) -> Bool {
        let columns = [
            matrix.columns.0,
            matrix.columns.1,
            matrix.columns.2,
            matrix.columns.3,
        ]
        return columns.allSatisfy { column in
            column.x.isFinite && column.y.isFinite &&
            column.z.isFinite && column.w.isFinite
        }
    }


    func reset() {
        pendingSubmission = nil
        adaptiveDensity.reset()
        entity.isEnabled = false
    }

    private func logStatsIfNeeded(
        frame: PointCloudFrame,
        renderedPointCount: Int
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
            "input_points=\(frame.pointCount) " +
            "rendered_points=\(renderedPointCount) " +
            "mode=\(adaptiveDensity.modeName) " +
            "submitted=\(submittedFrames) " +
            "gpu_busy_drops=\(gpuBusyDrops) " +
            String(
                format: "latency_avg=%.2fms latency_p95=%.2fms ",
                adaptiveDensity.averageLatencyMilliseconds,
                adaptiveDensity.p95LatencyMilliseconds
            ) +
            colorSummary
        )
    }

    private func logAdaptiveTransition() {
        dlog(
            "⚙️ [PointCloud GPU] Adaptive density → " +
            "\(adaptiveDensity.modeName) " +
            "(source_stride=\(adaptiveDensity.sourceStride), " +
            "thermal=\(String(describing: ProcessInfo.processInfo.thermalState)))"
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
