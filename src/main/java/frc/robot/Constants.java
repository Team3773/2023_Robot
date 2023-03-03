package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.42;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(28);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(32);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 6;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true; //true
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true; //true
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10; // CHANGED
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.54; // 9
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 2.84; // 11
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -2.04; // 10
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.28; // 12

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public final static class OperationConstants {
      public static final int armExtendMotorChannel = 10;
      public static final int armRotateMotorChannel = 9;
      public static final int elevatorMotorChannel = 11;
      public static final int clawMotorChannel = 12;

      public static final double kArmRotateGearRatio = 1/977;
      public static final double kArmExtendGearRatio = 1/380; // EDUCATED GUESS
      public static final double kElevatorGearRatio = 1/36;
      public static final double kClawGearRatio = 1/196;

      public static final double kArmRotateEncoderRot2Meter = 2 * Math.PI * kArmRotateGearRatio;
      public static final double kArmExtendEncoderRot2Meter = 2 * Math.PI * kArmExtendGearRatio;
      public static final double kElevatorEncoderRot2Meter = 2 * Math.PI * kElevatorGearRatio;
      public static final double kClawEncoderRot2Meter = 2 * Math.PI * kClawGearRatio;

      // CHANGE TO ACTUAL SETPOINTS
      public static final double kArmRotateSetpoint = 0;
      public static final double kArmExtendSetpoint = 0;
      public static final double kElevatorSetpoint = 0;
      public static final double kClawSetpoint = 0;

      // CHANGE VALUES BASED ON ENCODER READINGS
      public static final double kElevatorBottomPosition = -1.0;
      public static final double kElevatorTopPosition = 1.0;
      public static final double kElevatorMiddlePosition = 0.0;

      // EXTERNAL ENCODER PORTS
      public static final int karmExtendEncoderA = 0;
      public static final int karmExtendEncoderB = 1;
      public static final int karmRotateEncoderA = 2;
      public static final int karmRotateEncoderB = 3;

      public static final int limitSwitchPort = 4;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5; //1.5
        public static final double kPYController = 1.5; //1.5
        public static final double kPThetaController = 3; //3

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}
