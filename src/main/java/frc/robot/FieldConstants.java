package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants for points on the field
 * Relative to the blue side origin
 */
public final class FieldConstants {
    public final class BlueConstants {
        private BlueConstants() {}
        public static final Pose2d SPEAKER = new Pose2d(0, 5.55, new Rotation2d(0));
    }

    public final class RedConstants {
        private RedConstants() {}
        public static final Pose2d SPEAKER = new Pose2d(16.525, 5.55, new Rotation2d(0));
    }
}
