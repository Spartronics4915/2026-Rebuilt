package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;
import com.spartronics4915.frc2026.Constants.VisionConstants.VisionState;
import com.ctre.phoenix6.Utils;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem.RobotHeading;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Camera;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Limelight;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Luma;
import com.spartronics4915.frc2026.util.LimelightHelpers;
import com.spartronics4915.frc2026.util.LimelightHelpers.LimelightResults;
import com.spartronics4915.frc2026.util.LimelightHelpers.LimelightTarget_Retro;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    // TODO: Redo the way I calculate average ambiguity as it is currently for the tags not the pose

    private final static HashMap<String, Camera> cameras = new HashMap<>();

    private Luma luma;
    private Limelight limelight;

    private VisionSystemSim photonSim;
    private PhotonCameraSim lumaSim;
 
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonEstimator;
    private List<PhotonPipelineResult> photonPipelineResults;

    public static VisionState visionState = VisionState.GLOBAL;
    private boolean isSimulation;
    private boolean isDebugging;
 
    private static int tagCount;
    private static EstimatedRobotPose photonEstimatedPose;
    private static Pose2d photonPose;
    
    private int tagID;
    private double distanceToTag;
    private double trustedDistance;
    private double totalDistance;
    private double avgDistance;
    private int countTrustedTags;

    private List<Pose3d> trackedTagPoses = new ArrayList<>();
    private List<Pose3d> trustedTagPoses = new ArrayList<>();

    private double translationStdDevs;
    private double rotationStdDevs;
    private Matrix<N3, N1> currentStdDevs;

    public static double poseTimestamp;
    private final VisionConsumer consumer;

    private Supplier<Pose2d> pastPoseSupplier;
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private Supplier<RobotHeading> headingSupplier;

    private double visionError;
    private boolean isMultiTag;
    private double totalAmbiguity;
    private double avgAmbiguity;
    private boolean isGlobal;

    private static Camera localCamera;
    private Pose2d robotPose;

    private StructPublisher<Pose2d> rawPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Raw Vision Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> compensatedPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Compensated Vision Pose", Pose2d.struct).publish();
    private StructArrayPublisher<Pose3d> trackedTagsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Tracked Tags", Pose3d.struct).publish();
    private StructArrayPublisher<Pose3d> trustedTagsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Trusted Tags", Pose3d.struct).publish();
    private DoublePublisher visionErrorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Vision Error").publish();
    private BooleanPublisher visionPoseStrategyPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Is Multi Tag").publish();
    private IntegerPublisher amountOfTagsPublisher = NetworkTableInstance.getDefault().getIntegerTopic("Amount of tags").publish();
    private DoublePublisher visionAmbiguityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Vision Ambiguity").publish();
    private StringPublisher cameraNamePublisher = NetworkTableInstance.getDefault().getStringTopic("Camera Name").publish();
    private DoublePublisher translationStdDevsPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Translation Standard Deviation").publish();
    private DoublePublisher rotationStdDevsPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Rotation Standard Deviation").publish();
    private BooleanPublisher visionStatePublisher = NetworkTableInstance.getDefault().getBooleanTopic("Is Global").publish();

    private int leftSideTargets;
    private int rightSideTargets;
    private boolean majorityTargetSide;
    private boolean left = true;
    private boolean right = false;

    private double bestTx;
    private double bestTy;
    private double bestTa;
    private boolean bestSideDirection;

    private static LimelightResults colorResults;
    private static double tx;
    private static double ty;
    private static double ta;

    private BooleanPublisher targetLocationPublisher = NetworkTableInstance.getDefault().getBooleanTopic("Target Location").publish();

    public VisionSubsystem(
        VisionConsumer consumer, 
        Supplier<Pose2d> poseSupplier, 
        Supplier<Pose2d> pastPoseSupplier,
        Supplier<ChassisSpeeds> chassisSpeedSupplier,
        Supplier<RobotHeading> headingSupplier
    ) {
        this.consumer = consumer;
        this.poseSupplier = poseSupplier;
        this.pastPoseSupplier = pastPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.headingSupplier = headingSupplier;

        // Set up photon vision simulation:

        isSimulation = Robot.isSimulation();
        if (isSimulation) {
            photonSim = new VisionSystemSim("photon");
            photonSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
            PhotonCamera.setVersionCheckEnabled(false);
        }
        
        // Create cameras and add them to the camera map
        
        for (Camera camera : cameraList) {
            switch (camera.getType()) {
                case LUMA:
                    luma = new Luma(
                        camera.getName(), 
                        camera.getType(), 
                        camera.getPipelineIndex(),
                        camera.getLayout().get(), 
                        camera.getTransform().get()
                    );
                    cameras.put(camera.getName(), luma);
                    
                    if (isSimulation) {
                        lumaSim = new PhotonCameraSim(luma.getCamera().get(), simCameraProperties);
                        photonSim.addCamera(lumaSim, luma.getTransform().get());
                    }
                    
                    System.out.println("Camera Loaded: " + luma.getCamera().get().getName());
                    break;
            
                case LIMELIGHT:
                    limelight = new Limelight(
                        camera.getName(), 
                        camera.getType(),
                        camera.getPipelineIndex(),
                        camera.getX().get(), 
                        camera.getY().get(), 
                        camera.getZ().get(), 
                        camera.getYaw().get(), 
                        camera.getPitch().get(), 
                        camera.getRoll().get()
                    );

                    LimelightHelpers.setCameraPose_RobotSpace(
                        limelight.getName(),
                        limelight.getX().orElse(0.0), 
                        limelight.getY().orElse(0.0), 
                        limelight.getZ().orElse(0.0), 
                        limelight.getYaw().orElse(0.0), 
                        limelight.getPitch().orElse(0.0), 
                        limelight.getRoll().orElse(0.0)
                    );

                    LimelightHelpers.setPipelineIndex(
                        limelight.getName(), 
                        limelight.getPipelineIndex()
                    );
                    cameras.put(limelight.getName(), limelight);
                    break;
            }
        }    
    }
    //#region Color Detection 
    /*private void findRawTargetDirection(double[] rawColorTargets){
        leftSideTargets = 0;
        rightSideTargets = 0;
        for (int i = 0; i < rawColorTargets.length; i+= 3){
            if (rawColorTargets[i] < 0){
            leftSideTargets++;
            } else if (rawColorTargets[i] > 0 ){
                rightSideTargets++;
            }
        }
        if (leftSideTargets > rightSideTargets){
            targetLocationPublisher.accept(isLeftSideTarget);
        } else if (rightSideTargets > leftSideTargets){
            targetLocationPublisher.accept(!isLeftSideTarget);
        }  
    }*/
    /*private void basicTargetDirection(double tx){
        bestTx = LimelightHelpers.getTX(getName());
        if (bestTx < 0){
            targetLocationPublisher.accept(isLeftSideTarget);
        } else if (bestTx > 0){
            targetLocationPublisher.accept(isLeftSideTarget);
        }
    }*/


    private boolean getBestDirection(){
        for (LimelightTarget_Retro colorTarget: colorResults.targets_Retro){
            tx = colorTarget.tx;
            ty = colorTarget.ty;
            ta = colorTarget.ta;
            if (tx < 0){
                leftSideTargets++;
            } else if (tx > 0){
                rightSideTargets++;
            }
        }
        if (leftSideTargets > rightSideTargets){
            majorityTargetSide = left;
        } else if (rightSideTargets > leftSideTargets){
            majorityTargetSide = right;
        } else if (rightSideTargets == leftSideTargets){
            //still thinking about best way to execute this one
        }
        bestTx = LimelightHelpers.getTX(getName());
        bestTy = LimelightHelpers.getTY(getName());
        bestTa = LimelightHelpers.getTA(getName());
        if (bestTx > 0){
            bestSideDirection = right;
        } else if (bestTx < 0){
            bestSideDirection = left;
        }
        if (majorityTargetSide == bestSideDirection){
            targetLocationPublisher.accept(bestSideDirection);
        } else if (majorityTargetSide != bestSideDirection){
            if (ta > bestTa){
                targetLocationPublisher.accept(majorityTargetSide);
                bestSideDirection = majorityTargetSide;
            }
        }
        return bestSideDirection;
    }

    // This is where the magic happens:

    @Override
    public void periodic() {
        for (Camera camera : cameras.values()) {
            switch (camera.getType()) {
                case LUMA:
                    photonCamera = camera.getCamera().get();
                    photonEstimator = camera.getEstimator().get();

                    if (headingSupplier.get() != null) {
                        photonEstimator.addHeadingData(
                            headingSupplier.get().timestamp(), 
                            headingSupplier.get().rotation()
                        );
                    }

                    if (poseSupplier != null) robotPose = poseSupplier.get();

                    switch (visionState) {
                        case GLOBAL:
                            isGlobal = true;
                            photonPipelineResults = photonCamera.getAllUnreadResults();

                            if (photonPipelineResults.isEmpty() || photonPipelineResults == null) continue;

                            for (PhotonPipelineResult result : photonPipelineResults) {
                                if (!result.hasTargets()) continue;
                                tagCount = result.targets.size();
                                countTrustedTags = 0;

                                totalDistance = 0;
                                totalAmbiguity = 0;

                                trackedTagPoses.clear();
                                trustedTagPoses.clear();;

                                if (tagCount == 0) continue;
                            
                                switch (tagCount) {
                                    case 1:
                                        photonEstimatedPose = photonEstimator.estimateLowestAmbiguityPose(result).get();
                                        isMultiTag = false;
                                        break;
                                
                                    default:
                                        photonEstimatedPose = photonEstimator.estimateCoprocMultiTagPose(result).get();
                                        isMultiTag = true;
                                        break;
                                }

                                photonPose = photonEstimatedPose.estimatedPose.toPose2d();
                                if (robotPose.getTranslation().getDistance(photonPose.getTranslation()) > 0.5) continue;

                                for (PhotonTrackedTarget trackedTag : photonEstimatedPose.targetsUsed) {
                                    tagID = trackedTag.fiducialId;
                                    distanceToTag = trackedTag.getBestCameraToTarget().getTranslation().getNorm();
                                    
                                    if (tagCount == 1) trustedDistance = Units.feetToMeters(100);
                                        else trustedDistance = Units.feetToMeters(100);
                                    if (distanceToTag > trustedDistance) continue;
                                        else {
                                            countTrustedTags++;
                                            rebuiltApriltagFieldLayout.getTagPose(tagID).ifPresent(pose -> trustedTagPoses.add(pose));
                                        }

                                    totalDistance += distanceToTag;
                                    rebuiltApriltagFieldLayout.getTagPose(tagID).ifPresent(pose -> trackedTagPoses.add(pose));
                                    totalAmbiguity += trackedTag.poseAmbiguity;
                                }

                                // TODO: Implement my custom algorithm here and make sure to change stuff based off of single or multitag
                                avgAmbiguity = totalAmbiguity / tagCount;
                                avgDistance = totalDistance / tagCount;
                                switch (countTrustedTags) {
                                    case 0:
                                        continue;
                                    
                                    case 1:
                                        translationStdDevs = baseTransverseSingleTagStdDevs;
                                        rotationStdDevs = baseAngularSingleTagStdDevs;

                                        translationStdDevs += avgDistance * distancePunishment;
                                        translationStdDevs += avgAmbiguity * ambiguityPunishment;
                                        translationStdDevs += Math.sqrt(
                                            Math.pow(chassisSpeedSupplier.get().vxMetersPerSecond, 2) + 
                                            Math.pow(chassisSpeedSupplier.get().vyMetersPerSecond, 2)
                                        ) * transverseVelocityPunishment;
                                        break;
                                    
                                    default:
                                        translationStdDevs = baseTransverseMultiTagStdDevs;
                                        rotationStdDevs = baseAngularMultiTagStdDevs;
                                        
                                        translationStdDevs -= tagCount * tagReward;
                                        translationStdDevs += avgDistance * distancePunishment;
                                        translationStdDevs += avgAmbiguity * ambiguityPunishment;
                                        translationStdDevs += Math.sqrt(
                                            Math.pow(chassisSpeedSupplier.get().vxMetersPerSecond, 2) + 
                                            Math.pow(chassisSpeedSupplier.get().vyMetersPerSecond, 2)
                                        ) * transverseVelocityPunishment;
                                        
                                        rotationStdDevs += chassisSpeedSupplier.get().omegaRadiansPerSecond * angularVelocityPunishment;
                                        break;
                                }

                                poseTimestamp = Utils.fpgaToCurrentTime(photonEstimatedPose.timestampSeconds);
                                currentStdDevs = VecBuilder.fill(translationStdDevs, translationStdDevs, rotationStdDevs);
                                
                                this.consumer.accept(
                                    photonPose,
                                    poseTimestamp,
                                    currentStdDevs
                                );

                                trackedTagsPublisher.accept(trackedTagPoses.toArray(new Pose3d[0]));
                                trustedTagsPublisher.accept(trustedTagPoses.toArray(new Pose3d[0]));
                                
                                visionPoseStrategyPublisher.accept(isMultiTag);
                                amountOfTagsPublisher.accept(tagCount);
                                visionAmbiguityPublisher.accept(avgAmbiguity);
                            }
                            break;   

                        case LOCAL:
                            // TODO: In sim this is being funky, need to do some more testing
                            if (localCamera != camera) continue;
                            isGlobal = false;

                            photonPipelineResults = photonCamera.getAllUnreadResults();
                            if (photonPipelineResults.isEmpty() || photonPipelineResults == null) continue;

                            for (PhotonPipelineResult result : photonPipelineResults) {
                                if (!result.hasTargets()) continue;
                                
                                trackedTagPoses.clear();

                                photonEstimatedPose = photonEstimator.estimatePnpDistanceTrigSolvePose(result).get();

                                photonPose = photonEstimatedPose.estimatedPose.toPose2d();
                                if (robotPose.getTranslation().getDistance(photonPose.getTranslation()) > 0.5) continue;

                                translationStdDevs = localTransverseStdDevs;
                                rotationStdDevs = localAngularStdDevs;

                                currentStdDevs = VecBuilder.fill(translationStdDevs, translationStdDevs, rotationStdDevs);
                                poseTimestamp = Utils.fpgaToCurrentTime(photonEstimatedPose.timestampSeconds);

                                this.consumer.accept(
                                    photonPose, 
                                    poseTimestamp, 
                                    currentStdDevs
                                );

                                trackedTagPoses.add(rebuiltApriltagFieldLayout.getTagPose(result.getBestTarget().fiducialId).get());

                                visionAmbiguityPublisher.accept(result.getBestTarget().poseAmbiguity);
                                visionPoseStrategyPublisher.accept(false);
                                amountOfTagsPublisher.accept(1);
                                trackedTagsPublisher.accept(trackedTagPoses.toArray(new Pose3d[0]));
                                trustedTagsPublisher.accept(trackedTagPoses.toArray(new Pose3d[0]));
                            }
                            break;
                    }
                    
                    if (photonPose == null) continue;
                    visionError = Math.sqrt(Math.pow(poseSupplier.get().minus(photonPose).getX(), 2) + Math.pow(poseSupplier.get().minus(photonPose).getY(), 2));

                    rawPosePublisher.accept(photonPose);
                    compensatedPosePublisher.accept(pastPoseSupplier.get());
                    visionErrorPublisher.accept(visionError);
                    cameraNamePublisher.accept(camera.getName());
                    translationStdDevsPublisher.accept(translationStdDevs);
                    rotationStdDevsPublisher.accept(rotationStdDevs);
                    visionStatePublisher.accept(isGlobal);
                    break;
                // Naomi's domain:
                case LIMELIGHT:
                    colorResults = LimelightHelpers.getLatestResults(getName());
                    getBestDirection(); 
                break;
            }
        }
        if (isSimulation) {
            photonSim.update(poseSupplier.get());
        }
    }

    public static void setLocalCamera(String name) {
        localCamera = cameras.get(name);
    }

    // End of the magic :(

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }
}
