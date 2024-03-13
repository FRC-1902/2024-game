package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants for points on the field
 * Relative to the blue side origin
 */
public final class FieldConstants {
    public final class BlueConstants {
        public static final Pose2d SPEAKER = new Pose2d(0, 4.945, new Rotation2d(0)); // TODO: set me from michael, needed for auto speaker shooting
    }

    public final class RedConstants {
        public static final Pose2d SPEAKER = new Pose2d(0, 0, new Rotation2d(0)); // TODO: set me from michael, needed for auto speaker shooting
    }
}
