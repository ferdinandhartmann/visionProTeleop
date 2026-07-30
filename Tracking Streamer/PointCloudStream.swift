import Foundation
import simd

struct PointCloudFrame: Sendable {
    static let maximumPointCount = 100_000
    static let recordBytes = 9

    let sequence: UInt32
    let timestampNanoseconds: UInt64
    let pointCount: Int
    let boundsMin: SIMD3<Float>
    let boundsMax: SIMD3<Float>
    let packedPoints: Data
}

final class PointCloudChunkReassembler: @unchecked Sendable {
    private static let magic = Data([0x50, 0x43, 0x44, 0x32]) // "PCD2"
    private static let version: UInt8 = 2
    private static let flags: UInt8 = 0x03
    private static let headerBytes = 60
    private static let chunkPayloadBytes = 48 * 1024 - headerBytes
    private static let pointsPerChunk =
        chunkPayloadBytes / PointCloudFrame.recordBytes
    private static let assemblyTimeout = 0.1

    private final class Assembly {
        let sequence: UInt32
        let timestampNanoseconds: UInt64
        let pointCount: Int
        let chunkCount: Int
        let boundsMin: SIMD3<Float>
        let boundsMax: SIMD3<Float>
        let startedAt: TimeInterval
        var received: [Bool]
        var receivedCount: Int
        var payload: Data

        init(
            sequence: UInt32,
            timestampNanoseconds: UInt64,
            pointCount: Int,
            chunkCount: Int,
            boundsMin: SIMD3<Float>,
            boundsMax: SIMD3<Float>,
            startedAt: TimeInterval
        ) {
            self.sequence = sequence
            self.timestampNanoseconds = timestampNanoseconds
            self.pointCount = pointCount
            self.chunkCount = chunkCount
            self.boundsMin = boundsMin
            self.boundsMax = boundsMax
            self.startedAt = startedAt
            self.received = Array(repeating: false, count: chunkCount)
            self.receivedCount = 0
            self.payload = Data(
                count: pointCount * PointCloudFrame.recordBytes
            )
        }
    }

    private let queue = DispatchQueue(
        label: "visionpro.pointcloud.reassembly",
        qos: .userInteractive
    )
    private var assembly: Assembly?
    private var lastCompletedSequence: UInt32?
    private var completedFrames = 0
    private var rejectedChunks = 0
    private var incompleteFrames = 0
    private var lastStatsLog = 0.0

    var onFrame: (@Sendable (PointCloudFrame) -> Void)?

    func receive(_ data: Data) {
        queue.async { [weak self] in
            self?.consume(data)
        }
    }

    func reset() {
        queue.async { [weak self] in
            self?.assembly = nil
            self?.lastCompletedSequence = nil
        }
    }

    private func consume(_ data: Data) {
        guard let header = parseHeader(data) else {
            rejectChunk()
            return
        }
        if let lastCompletedSequence,
           !isNewer(header.sequence, than: lastCompletedSequence) {
            return
        }

        let now = ProcessInfo.processInfo.systemUptime
        if let current = assembly, now - current.startedAt > Self.assemblyTimeout {
            assembly = nil
            incompleteFrames += 1
        }

        if let current = assembly, current.sequence != header.sequence {
            guard isNewer(header.sequence, than: current.sequence) else {
                return
            }
            incompleteFrames += current.receivedCount == current.chunkCount ? 0 : 1
            assembly = nil
        }

        if assembly == nil {
            assembly = Assembly(
                sequence: header.sequence,
                timestampNanoseconds: header.timestampNanoseconds,
                pointCount: header.pointCount,
                chunkCount: header.chunkCount,
                boundsMin: header.boundsMin,
                boundsMax: header.boundsMax,
                startedAt: now
            )
        }

        guard let current = assembly,
              current.sequence == header.sequence,
              current.timestampNanoseconds == header.timestampNanoseconds,
              current.pointCount == header.pointCount,
              current.chunkCount == header.chunkCount,
              current.boundsMin == header.boundsMin,
              current.boundsMax == header.boundsMax else {
            rejectChunk()
            return
        }

        if current.received[header.chunkIndex] {
            return
        }

        let payloadStart = header.headerBytes
        let destinationStart = header.firstPoint * PointCloudFrame.recordBytes
        let payloadLength = header.chunkPointCount * PointCloudFrame.recordBytes
        current.payload.replaceSubrange(
            destinationStart..<(destinationStart + payloadLength),
            with: data[payloadStart..<(payloadStart + payloadLength)]
        )
        current.received[header.chunkIndex] = true
        current.receivedCount += 1

        if current.receivedCount == current.chunkCount {
            let frame = PointCloudFrame(
                sequence: current.sequence,
                timestampNanoseconds: current.timestampNanoseconds,
                pointCount: current.pointCount,
                boundsMin: current.boundsMin,
                boundsMax: current.boundsMax,
                packedPoints: current.payload
            )
            assembly = nil
            lastCompletedSequence = current.sequence
            completedFrames += 1
            logStatsIfNeeded()
            onFrame?(frame)
        }
    }

    private struct Header {
        let headerBytes: Int
        let sequence: UInt32
        let timestampNanoseconds: UInt64
        let pointCount: Int
        let chunkIndex: Int
        let chunkCount: Int
        let firstPoint: Int
        let chunkPointCount: Int
        let boundsMin: SIMD3<Float>
        let boundsMax: SIMD3<Float>
    }

    private func parseHeader(_ data: Data) -> Header? {
        guard data.count >= Self.headerBytes,
              data.prefix(4) == Self.magic,
              data[4] == Self.version,
              data[5] == Self.flags else {
            return nil
        }

        let headerBytes = Int(readUInt16(data, at: 6))
        let sequence = readUInt32(data, at: 8)
        let timestamp = readUInt64(data, at: 12)
        let pointCount = Int(readUInt32(data, at: 20))
        let chunkIndex = Int(readUInt16(data, at: 24))
        let chunkCount = Int(readUInt16(data, at: 26))
        let firstPoint = Int(readUInt32(data, at: 28))
        let chunkPointCount = Int(readUInt32(data, at: 32))
        let boundsMin = SIMD3<Float>(
            readFloat(data, at: 36),
            readFloat(data, at: 40),
            readFloat(data, at: 44)
        )
        let boundsMax = SIMD3<Float>(
            readFloat(data, at: 48),
            readFloat(data, at: 52),
            readFloat(data, at: 56)
        )

        let payloadBytes = chunkPointCount.multipliedReportingOverflow(
            by: PointCloudFrame.recordBytes
        )
        let expectedChunkCount =
            (pointCount + Self.pointsPerChunk - 1) / Self.pointsPerChunk
        let expectedFirstPoint = chunkIndex * Self.pointsPerChunk
        let expectedChunkPointCount = min(
            Self.pointsPerChunk,
            pointCount - expectedFirstPoint
        )
        guard headerBytes == Self.headerBytes,
              pointCount > 0,
              pointCount <= PointCloudFrame.maximumPointCount,
              chunkCount == expectedChunkCount,
              chunkIndex >= 0,
              chunkIndex < chunkCount,
              firstPoint == expectedFirstPoint,
              chunkPointCount == expectedChunkPointCount,
              !payloadBytes.overflow,
              data.count == headerBytes + payloadBytes.partialValue,
              isFinite(boundsMin),
              isFinite(boundsMax),
              boundsMin.x <= boundsMax.x,
              boundsMin.y <= boundsMax.y,
              boundsMin.z <= boundsMax.z else {
            return nil
        }

        return Header(
            headerBytes: headerBytes,
            sequence: sequence,
            timestampNanoseconds: timestamp,
            pointCount: pointCount,
            chunkIndex: chunkIndex,
            chunkCount: chunkCount,
            firstPoint: firstPoint,
            chunkPointCount: chunkPointCount,
            boundsMin: boundsMin,
            boundsMax: boundsMax
        )
    }

    private func rejectChunk() {
        rejectedChunks += 1
        logStatsIfNeeded()
    }

    private func logStatsIfNeeded() {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastStatsLog >= 1.0 else { return }
        lastStatsLog = now
        dlog(
            "🌫️ [PointCloud] complete=\(completedFrames) " +
            "rejected_chunks=\(rejectedChunks) incomplete=\(incompleteFrames)"
        )
    }

    private func isNewer(_ candidate: UInt32, than current: UInt32) -> Bool {
        Int32(bitPattern: candidate &- current) > 0
    }

    private func isFinite(_ value: SIMD3<Float>) -> Bool {
        value.x.isFinite && value.y.isFinite && value.z.isFinite
    }

    private func readUInt16(_ data: Data, at offset: Int) -> UInt16 {
        data.withUnsafeBytes {
            UInt16(littleEndian: $0.loadUnaligned(fromByteOffset: offset, as: UInt16.self))
        }
    }

    private func readUInt32(_ data: Data, at offset: Int) -> UInt32 {
        data.withUnsafeBytes {
            UInt32(littleEndian: $0.loadUnaligned(fromByteOffset: offset, as: UInt32.self))
        }
    }

    private func readUInt64(_ data: Data, at offset: Int) -> UInt64 {
        data.withUnsafeBytes {
            UInt64(littleEndian: $0.loadUnaligned(fromByteOffset: offset, as: UInt64.self))
        }
    }

    private func readFloat(_ data: Data, at offset: Int) -> Float {
        Float(bitPattern: readUInt32(data, at: offset))
    }
}
