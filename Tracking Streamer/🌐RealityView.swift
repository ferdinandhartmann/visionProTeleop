import SwiftUI
import RealityKit
import ARKit

struct 🌐RealityView: View {
    var model: 🥽AppModel
    @State private var isMinimized = false
    @State private var showViewControls = false
    @State private var previewZDistance: Float? = nil
    @State private var previewActive = false
    @State private var userInteracted = false
    @State private var previewStatusPosition: (x: Float, y: Float)? = nil
    @State private var previewStatusActive = false
    @State private var fixedStatusTransform: Transform? = nil
    @ObservedObject private var dataManager = DataManager.shared
    
    var body: some View {
        RealityView { content, attachments in
            dlog("🟢 [🌐RealityView] RealityView content block called")
            
            let resultLabelEntity = attachments.entity(for: Self.resultLabelID)!
            resultLabelEntity.components.set(🧑HeadTrackingComponent())
            resultLabelEntity.name = 🧩Name.resultLabel
            
            // Create status display anchor
            let statusAnchor = AnchorEntity(.head)
            statusAnchor.name = "statusHeadAnchor"
            content.add(statusAnchor)
            
            let statusWorldAnchor = AnchorEntity(world: .zero)
            statusWorldAnchor.name = "statusWorldAnchor"
            content.add(statusWorldAnchor)
            
            // Create status container entity
            let statusContainer = Entity()
            statusContainer.name = "statusContainer"
            statusContainer.setParent(statusAnchor)
            
            // Position status in top area (relative to head)
            statusContainer.transform.translation = SIMD3<Float>(0.0, 0.0, -1.0)
            
            // Attach the status UI to the container
            if let statusAttachment = attachments.entity(for: Self.statusAttachmentID) {
                dlog("🟢 [🌐RealityView] Status attachment found and attached")
                statusAttachment.setParent(statusContainer)
            } else {
                dlog("🔴 [🌐RealityView] Status attachment NOT found!")
            }
            
            // Create preview status container entity (initially hidden)
            let statusPreviewContainer = Entity()
            statusPreviewContainer.name = "statusPreviewContainer"
            statusPreviewContainer.setParent(statusAnchor)
            
            // Initialize at the correct Z position
            statusPreviewContainer.transform.translation = SIMD3<Float>(
                dataManager.statusMinimizedXPosition,
                dataManager.statusMinimizedYPosition,
                -1.0
            )
            
            // Attach the status preview UI to the preview container
            if let statusPreviewAttachment = attachments.entity(for: "statusPreview") {
                dlog("🟢 [🌐RealityView] Status preview attachment found and attached")
                statusPreviewAttachment.setParent(statusPreviewContainer)
                statusPreviewContainer.isEnabled = false
            }
        } update: { updateContent, attachments in
            // Explicitly depend on state to trigger updates
            let _ = isMinimized
            let _ = showViewControls
            let _ = previewStatusPosition
            let _ = previewStatusActive
            let _ = dataManager.statusMinimizedXPosition
            let _ = dataManager.statusMinimizedYPosition
            let _ = dataManager.statusFixedToWorld
            
            func findEntity(named name: String, in collection: RealityViewEntityCollection) -> Entity? {
                for entity in collection {
                    if entity.name == name { return entity }
                    if let nested = entity.findEntity(named: name) { return nested }
                }
                return nil
            }
            
            // Update status container position based on minimized state
            if let statusAnchor = findEntity(named: "statusHeadAnchor", in: updateContent.entities) as? AnchorEntity,
               let statusWorldAnchor = findEntity(named: "statusWorldAnchor", in: updateContent.entities) as? AnchorEntity,
               let statusContainer = findEntity(named: "statusContainer", in: updateContent.entities) {
                let isStatusFixed = dataManager.statusFixedToWorld
                if isStatusFixed, statusContainer.parent !== statusWorldAnchor {
                    statusContainer.setParent(statusWorldAnchor, preservingWorldTransform: true)
                } else if !isStatusFixed, statusContainer.parent !== statusAnchor {
                    statusContainer.setParent(statusAnchor, preservingWorldTransform: true)
                }
                
                // When minimized, use custom position; when maximized, use (0, 0, -1.0)
                let targetTranslation: SIMD3<Float>
                if isMinimized {
                    targetTranslation = SIMD3<Float>(
                        dataManager.statusMinimizedXPosition,
                        dataManager.statusMinimizedYPosition,
                        -1.0
                    )
                } else {
                    // Maximized stays at (0, 0, -1.0)
                    targetTranslation = SIMD3<Float>(0.0, 0.0, -1.0)
                }
                
                if isStatusFixed, let lockedTransform = fixedStatusTransform {
                    statusContainer.move(to: lockedTransform, relativeTo: statusContainer.parent, duration: 0.1, timingFunction: .linear)
                } else {
                    // Animate the position change
                    var transform = statusContainer.transform
                    transform.translation = targetTranslation
                    statusContainer.move(to: transform, relativeTo: statusContainer.parent, duration: 0.5, timingFunction: .easeInOut)
                }
            }
            
            // Handle status preview
            if let statusAnchor = findEntity(named: "statusHeadAnchor", in: updateContent.entities) as? AnchorEntity,
               let statusWorldAnchor = findEntity(named: "statusWorldAnchor", in: updateContent.entities) as? AnchorEntity,
               let statusPreviewContainer = findEntity(named: "statusPreviewContainer", in: updateContent.entities) {
                let isStatusFixed = dataManager.statusFixedToWorld
                
                if isStatusFixed, statusPreviewContainer.parent !== statusWorldAnchor {
                    statusPreviewContainer.setParent(statusWorldAnchor, preservingWorldTransform: true)
                } else if !isStatusFixed, statusPreviewContainer.parent !== statusAnchor {
                    statusPreviewContainer.setParent(statusAnchor, preservingWorldTransform: true)
                }
                
                let shouldShowPreview = previewStatusPosition != nil || previewStatusActive
                
                if shouldShowPreview {
                    let xPos = previewStatusPosition?.x ?? dataManager.statusMinimizedXPosition
                    let yPos = previewStatusPosition?.y ?? dataManager.statusMinimizedYPosition
                    
                    statusPreviewContainer.isEnabled = true
                    var previewTransform = statusPreviewContainer.transform
                    previewTransform.translation = SIMD3<Float>(xPos, yPos, -1.0)
                    statusPreviewContainer.move(to: previewTransform, relativeTo: statusPreviewContainer.parent, duration: 0.1, timingFunction: .linear)
                } else {
                    statusPreviewContainer.isEnabled = false
                }
            }
        } attachments: {
            Attachment(id: Self.resultLabelID) {
            }
            Attachment(id: Self.statusAttachmentID) {
                dlog("🟡 [🌐RealityView] Status attachment builder called")
                return StatusOverlay(
                    showVideoStatus: false, 
                    isMinimized: $isMinimized,
                    showViewControls: $showViewControls,
                    previewZDistance: $previewZDistance,
                    previewActive: $previewActive,
                    userInteracted: $userInteracted,
                    videoFixed: .constant(false),
                    statusFixed: Binding(
                        get: { dataManager.statusFixedToWorld },
                        set: { newValue in dataManager.statusFixedToWorld = newValue }
                    ),
                    previewStatusPosition: $previewStatusPosition,
                    previewStatusActive: $previewStatusActive
                )
            }
            
            Attachment(id: "statusPreview") {
                StatusPreviewView(
                    showVideoStatus: false,
                    videoFixed: false,
                    statusFixed: dataManager.statusFixedToWorld
                )
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
        )
        .task { self.model.run() }
        .task { await self.model.processDeviceAnchorUpdates() }
        // .task { self.model.startserver() }
        .task(priority: .low) { await self.model.processReconstructionUpdates() }
        .onChange(of: dataManager.statusFixedToWorld) { _, isFixed in
            if isFixed {
                let headWorldMatrix = DataManager.shared.latestHandTrackingData.Head
                let targetTranslation: SIMD3<Float>
                if isMinimized {
                    targetTranslation = SIMD3<Float>(
                        dataManager.statusMinimizedXPosition,
                        dataManager.statusMinimizedYPosition,
                        -1.0
                    )
                } else {
                    targetTranslation = SIMD3<Float>(0.0, 0.0, -1.0)
                }
                var offsetTransform = Transform()
                offsetTransform.translation = targetTranslation
                let worldMatrix = simd_mul(headWorldMatrix, offsetTransform.matrix)
                fixedStatusTransform = Transform(matrix: worldMatrix)
            } else {
                fixedStatusTransform = nil
            }
        }
        .upperLimbVisibility(dataManager.upperLimbVisible ? .visible : .hidden)
    }
    static let resultLabelID: String = "resultLabel"
    static let statusAttachmentID: String = "status"
}
