package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public class DriveConstants {

        public static final boolean fieldRelative = true;

        public static final int frontRightDriveCANId = 19;
        public static final int frontLeftDriveCANId = 0;
        public static final int backRightDriveCANId = 10;
        public static final int backLeftDriveCANId = 9;

        public static final int frontRightTurnCANId = 18;
        public static final int frontLeftTurnCANId = 1;
        public static final int backRightTurnCANId = 11;
        public static final int backLeftTurnCANId = 8;

        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0.01;

        public static final double kPAngular = 1;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0;

        public static final double llkP = .2;
        public static final double llkI = 0;
        public static final double llkD = 0;

        public static final double maxVelocityMetersPerSec = 4.3;
        public static final double maxAccelerationMetersPerSec2 = 5; // TODO: make this a real number

        public static final double kTrackWidth = Units.inchesToMeters(20.75);
        public static final double kWheelBase = Units.inchesToMeters(20.75);
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d((kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((kWheelBase / 2) - .0508, -kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2) - .0508, -kTrackWidth / 2)
            );

        public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

        public static final double angleTolerance = 5;
        public static final double velocityTolerance = 0.8;
        public static final double positionTolerance = 0.08;
        public static final double xandyvelocityTolerance = 0.08;

        //Vision constants
        public static Transform3d rightCameraLocation = new Transform3d(-0.2707, 0.1775, 0.3732, new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(190)));
        public static Transform3d leftCameraLocation = new Transform3d(-0.2707, -0.1775, 0.3732, new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(170)));
        public static double maxVisionDistanceTolerance = 5;// for the max distance between cam and tag in meters
        public static double maxAmbiguity = 1; // max ambiguity out of 1
        public static double zTolerance = 0.25;
        public static double rollPitchTolerance = Units.degreesToRadians(10);
        public static double visionPoseDiffTolerance = 99; // for the diff between estimated vision pose and odometry in
                                                          // meters
    }

    public class OperatorConstants {

        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; // TODO: find good deadzone values for the xbox controllers

        public enum DriveType {
            JOYSTICK,
            XBOX,
        }
    }
}
