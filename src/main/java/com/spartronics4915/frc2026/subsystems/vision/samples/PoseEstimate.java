package com.spartronics4915.frc2026.subsystems.vision.samples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record PoseEstimate(
    Pose2d fieldToRobot,
    double timestampSeconds,
    double latency,
    double avgTagArea,
    double quality,
    int[] fiducialIds
) implements StructSerializable {

    public PoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = new Pose2d();
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
    public static PoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = new Pose2d();
        }

        int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
        for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
            if (poseEstimate.rawFiducials[i] != null) {
                fiducialIds[i] = poseEstimate.rawFiducials[i].id;
            }
        }

        return new PoseEstimate(
            fieldToRobot,
            poseEstimate.timestampSeconds,
            poseEstimate.latency,
            poseEstimate.avgTagArea,
            fiducialIds.length > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity,
            fiducialIds
        );
    }

    /** Converts a Photon pipeline result to a MegatagPoseEstimate. */
    public static PoseEstimate fromPhotonCamera(EstimatedRobotPose poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.estimatedPose.toPose2d();
        if (fieldToRobot == null) {
            fieldToRobot = new Pose2d();
        }

        int[] fiducialIds = new int[poseEstimate.targetsUsed.size()];
        double totalTagArea = 0.0;
        for (int i = 0; i < poseEstimate.targetsUsed.size(); i++) {
            if (poseEstimate.targetsUsed.get(i) != null) {
                fiducialIds[i] = poseEstimate.targetsUsed.get(i).fiducialId;
                totalTagArea += poseEstimate.targetsUsed.get(i).area;
            }
        }

        return new PoseEstimate(
            fieldToRobot,
            poseEstimate.timestampSeconds,
            Utils.getCurrentTimeSeconds() - poseEstimate.timestampSeconds,
            totalTagArea / poseEstimate.targetsUsed.size(),
            fiducialIds.length > 1 ? 1.0 : 1.0 - poseEstimate.targetsUsed.get(0).poseAmbiguity,
            fiducialIds
        );
    }

    public static final PoseEstimateStruct struct = new PoseEstimateStruct();

    public static class PoseEstimateStruct implements Struct<PoseEstimate> {
        @Override
        public Class<PoseEstimate> getTypeClass() {
            return PoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "record: MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + 3 * Double.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot; double timestampSeconds; double latency; double avgTagArea";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public PoseEstimate unpack(ByteBuffer buffer) {
            return new PoseEstimate(
                Pose2d.struct.unpack(buffer), 
                buffer.getDouble(), 
                buffer.getDouble(), 
                buffer.getDouble(), 
                buffer.getDouble(), 
                new int[0]
            );
        }

        @Override
        public void pack(ByteBuffer buffer, PoseEstimate value) {
            Pose2d.struct.pack(buffer, value.fieldToRobot());
            buffer.putDouble(value.timestampSeconds());
            buffer.putDouble(value.latency());
            buffer.putDouble(value.avgTagArea());
            buffer.putDouble(value.quality());
        }

        @Override
        public String getTypeName() {
            return "PoseEstimate";
        }
    }
    
}