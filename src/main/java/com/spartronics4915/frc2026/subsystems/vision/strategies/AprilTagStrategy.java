package com.spartronics4915.frc2026.subsystems.vision.strategies;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext.VisionState;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagStrategy implements PipelineStrategyInterface {

    @Override
    public String getStrategyName() {
        return "AprilTag";
    }

    @Override
    public StrategyResult process(List<CameraResult> results, VisionContext context) {
        VisionState state = context.getCurrentState();
        
        //if (state == VisionState.IDLE) return StrategyResult.empty();

        List<PoseEstimate> poseEstimates = new ArrayList<>();

        for (CameraResult result : results) {
            if (!result.hasPose()) continue;
            switch (state) {
                case GLOBAL:
                    processGlobalResult(result, poseEstimates, context);
                    break;
                
                case LOCAL:
                    processLocalResult(result, poseEstimates, context);
                    break;
                
                default:
                    break;
            }
        }

        return StrategyResult.fromPoses(poseEstimates);
    }

    private void processGlobalResult(
        CameraResult result,
        List<PoseEstimate> poseEstimates,
        VisionContext context
    ) {
        
        VisionConfiguration config = context.getConfig();
        
        if (result.getTargetCount() < 2) {
            if (result.getAverageDistanceToTargets() > config.maxSingleTagDistanceMeters * 0.5 ||
                result.getAmbiguity() > 0.1) {
                return;
            }
        }

        double maxDistance = (result.getTargetCount() == 1) 
            ? config.maxSingleTagDistanceMeters 
            : config.maxMultiTagDistanceMeters;
        
        if (result.getAverageDistanceToTargets() > maxDistance) return;

        if (result.getAmbiguity() > config.maxAmbiguityScore) return;

        if (config.enableHistoricalValidation) {
            if (!isPhysicallyPlausible(result.getEstimatedPose().get(), context)) return;
        }

        PoseEstimate estimate = new PoseEstimate(
            result.getEstimatedPose().get(),
            result.getTimestampSeconds(),
            result.getStdDevs(),
            result.getCameraName() + "-global"
        );

        poseEstimates.add(estimate);
    }

    private void processLocalResult(
        CameraResult result,
        List<PoseEstimate> poseEstimates,
        VisionContext context
    ) {
        
        VisionConfiguration config = context.getConfig();

        double maxDistance = config.maxSingleTagDistanceMeters * 1.5;

        if (result.getAverageDistanceToTargets() > maxDistance) return;

        PoseEstimate estimate = new PoseEstimate(
            result.getEstimatedPose().get(),
            result.getTimestampSeconds(),
            config.localStdDevs,
            result.getCameraName() + "-local"
        );

        poseEstimates.add(estimate);
    }

    private boolean isPhysicallyPlausible(Pose2d visionPose, VisionContext context) {
        VisionConfiguration config = context.getConfig();
        Pose2d currentPose = context.getCurrentRobotPose();

        double dx = visionPose.getX() - currentPose.getX();
        double dy = visionPose.getY() - currentPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > config.maxPoseJumpMeters) return false;

        return true;
    }
}
