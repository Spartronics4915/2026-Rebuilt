package com.spartronics4915.frc2026.subsystems.vision.strategies;

import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface PipelineStrategyInterface {
    
    StrategyResult process(List<CameraResult> results, VisionContext context);
    String getStrategyName();

    class StrategyResult {
        private final List<PoseEstimate> poseEstimates;
        private final List<DetectedObject> detectedObjects;
        private final boolean successful;

        public StrategyResult(
            List<PoseEstimate> poseEstimates,
            List<DetectedObject> detectedObjects,
            boolean successful
        ) {
            this.poseEstimates = List.copyOf(poseEstimates);
            this.detectedObjects = List.copyOf(detectedObjects);
            this.successful = successful;
        }

        public List<PoseEstimate> getPoseEstimates() {
            return poseEstimates;
        }

        public List<DetectedObject> getDetectedObjects() {
            return detectedObjects;
        }

        public boolean isSuccessful() {
            return successful;
        }

        public static StrategyResult empty() {
            return new StrategyResult(List.of(), List.of(), false);
        }

        public static StrategyResult fromPoses(List<PoseEstimate> poses) {
            return new StrategyResult(poses, List.of(), !poses.isEmpty());
        }

        public static StrategyResult fromObjects(List<DetectedObject> objects) {
            return new StrategyResult(List.of(), objects, !objects.isEmpty());
        }
    }

    class DetectedObject {
        private final Pose2d fieldPosition;
        private final String className;
        private final double confidence;
        private final double timestamp;

        public DetectedObject(Pose2d fieldPosition, String className, double confidence, double timestamp) {
            this.fieldPosition = fieldPosition;
            this.className = className;
            this.confidence = confidence;
            this.timestamp = timestamp;
        }

        public Pose2d getFieldPosition() {
            return fieldPosition;
        }

        public String getClassName() {
            return className;
        }

        public double getConfidence() {
            return confidence;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    class PoseEstimate {
        private final Pose2d pose;
        private final double timestamp;
        private final Matrix<N3, N1> stdDevs;
        private final String source;

        public PoseEstimate(
            Pose2d pose,
            double timestamp,
            Matrix<N3, N1> stdDevs,
            String source
        ) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDevs = stdDevs;
            this.source = source;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Matrix<N3, N1> getStdDevs() {
            return stdDevs;
        }

        public String getSource() {
            return source;
        }
    }
}
