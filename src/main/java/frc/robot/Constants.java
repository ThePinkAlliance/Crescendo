// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.Gains;

/** Add your docs here. */
public class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.80);
        public static final double kDriveMotorGearRatio = 0.1633986928;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.47;

        public static final Gains kBackLeftSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kBackRightSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kFrontRightSteerGains = new Gains(.34, 0.0, 0);
        public static final Gains kFrontLeftSteerGains = new Gains(.34, 0.0, 0);

    }

    public static final class OIConstants {
        public static final double kJoystickDeadband = 0.05;
    }

    public static final class FieldConstants {
        public static final AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23.25);

        public static final double kBaseRadius = Math.sqrt((kTrackWidth * kTrackWidth) + (kWheelBase * kWheelBase));

        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 17;
        public static final int kBackLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 15;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 18;
        public static final int kBackLeftTurningMotorPort = 14;
        public static final int kFrontRightTurningMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningReversed = false;
        public static final boolean kBackLeftTurningReversed = false;
        public static final boolean kFrontRightTurningReversed = false;
        public static final boolean kBackRightTurningReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 8;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 6;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /**
         * These values where determined by lining up all the wheels and recording the
         * outputed positions.
         */
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -1.52;// 2.688;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.47;// -1.7185;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.97;// 0.182;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.92;// -2.519;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.437;// 2.91;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 18; // 18 rad/sec

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; // 0.96
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static double kTeleDriveSpeedReduction = 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.55;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.5;
    }

    public static final class RobotConstants {
        public enum RobotType {
            ROBOT_ONE,
            ROBOT_TWO
        }

        public static RobotType CURRENT_ROBOT = RobotType.ROBOT_TWO;
    }
}
