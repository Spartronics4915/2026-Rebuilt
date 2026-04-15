package com.spartronics4915.frc2026.subsystems.vision.samples;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.util.vision.LimelightHelpers;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 *
 * @param id The fiducial marker ID
 * @param tx Normalized horizontal offset (-1 to 1)
 * @param ty Normalized vertical offset (-1 to 1)
 * @param ambiguity Pose ambiguity score (0 = confident, 1 = ambiguous)
 * @param area Target area as percentage of image
 */
public record FiducialObservation(int id, double tx, double ty, double ambiguity, double area) implements StructSerializable {

    /** Converts a Limelight raw fiducial to a FiducialObservation. */
    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        if (fiducial == null) {
            return null;
        }
        return new FiducialObservation(
            fiducial.id, 
            fiducial.txnc, 
            fiducial.tync, 
            fiducial.ambiguity, 
            fiducial.ta
        );
    }

    /** Converts an array of Limelight raw fiducials to FiducialObservation array. */
    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        if (fiducials == null) {
            return new FiducialObservation[0];
        }
        return Arrays.stream(fiducials)
            .map(FiducialObservation::fromLimelight)
            .filter(Objects::nonNull)
            .toArray(FiducialObservation[]::new);
    }
    
    /**
     * Converts a list of PhotonLib tracked targets into an array of FiducialObservations.
     */
    public static FiducialObservation[] fromPhotonCamera(List<PhotonTrackedTarget> fiducials) {
        if (fiducials == null) {
            return new FiducialObservation[0]; // Return empty array instead of null to prevent NPEs
        }
        return fiducials.stream()
            .map(FiducialObservation::fromPhotonCamera) // Assuming a method: fromPhotonCamera(PhotonTrackedTarget t)
            .filter(Objects::nonNull)
            .toArray(FiducialObservation[]::new);
    }

    /**
     * Converts a single PhotonTrackedTarget to a FiducialObservation.
     */
    public static FiducialObservation fromPhotonCamera(PhotonTrackedTarget target) {
        if (target == null) return null;

        return new FiducialObservation(
            target.fiducialId,
            target.yaw,
            target.skew,
            target.poseAmbiguity,
            target.area
        );
    }

    public static final Struct<FiducialObservation> struct = 
        new Struct<FiducialObservation>() {
            @Override
            public Class<FiducialObservation> getTypeClass() {
                return FiducialObservation.class;
            }

            @Override
            public String getTypeString() {
                return "record:FiducialObservation";
            }

            @Override
            public int getSize() {
                return Integer.BYTES + 4 * Double.BYTES;
            }

            @Override
            public String getSchema() {
                return "int id; double tx; double ty; double ambiguity";
            }

            @Override
            public FiducialObservation unpack(ByteBuffer buffer) {
                int id = buffer.getInt();
                double tx = buffer.getDouble();
                double ty = buffer.getDouble();
                double ambiguity = buffer.getDouble();
                double area = buffer.getDouble();
                return new FiducialObservation(id, tx, ty, ambiguity, area);
            }

            @Override
            public void pack(ByteBuffer buffer, FiducialObservation value) {
                buffer.putInt(value.id());
                buffer.putDouble(value.tx());
                buffer.putDouble(value.ty());
                buffer.putDouble(value.ambiguity());
                buffer.putDouble(value.area());
            }

            @Override
            public String getTypeName() {
                return "FiducialObservation";
            }
        };
}