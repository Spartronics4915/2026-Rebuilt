package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface ResultInterface {

    //#region ------ Getters ------

    String getSourceName();
    double getTimestampSeconds();
    double getLatencyMs();
    Pose2d getPose();
    Matrix<N3, N1> getStdDevs();
    List<PhotonTrackedTarget> getTargets();
    int getTargetCount();
    double getAverageDistanceToTargets();
    double getAmbiguity();
    double getAverageArea();
    double getXAnisotropy();
    double getYAnisotropy();
    ChassisSpeeds getChassisSpeeds();
    
    //#endregion

}
