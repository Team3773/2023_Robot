package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperationConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
    };

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, gyro.getRotation2d(),
        modulePositions, new Pose2d(5.0, 13.5, new Rotation2d()));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void resetStates() 
    {        
        frontLeft.resetModuleStates();
        frontRight.resetModuleStates();
        backLeft.resetModuleStates();
        backRight.resetModuleStates();
    }

    public void zeroAbsEncodersSwerveSub()
    {
        System.out.println("ZEROOO SUB");
        frontLeft.zeroAbsEncoder();
        frontRight.zeroAbsEncoder();
        backLeft.zeroAbsEncoder();
        backRight.zeroAbsEncoder();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), modulePositions, pose);
    }

    @Override
    public void periodic() {
        odometer.update(gyro.getRotation2d(), modulePositions);
        frontLeft.toSmartDashboard();
        frontRight.toSmartDashboard();
        backLeft.toSmartDashboard();
        backRight.toSmartDashboard();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Pitch", getPitch());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // SwerveModuleState[] moduleStates = {};

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SwerveDriveKinematics.
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    
    public double getPitch()
    {
        return gyro.getPitch();
    }
    public boolean modulesarezero()
    {   
        System.out.println("TRYING");
        if((Math.abs(frontLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(frontLeft.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
        {
            frontLeft.stop();
        }
        if((Math.abs(frontRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(frontRight.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
        {
            frontRight.stop();
        }
        if((Math.abs(backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(backLeft.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
        {
            backLeft.stop();
        }
        if((Math.abs(backRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(backRight.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
        {
            backRight.stop();
        }

        if(
            ((Math.abs(frontLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(frontLeft.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
            &&((Math.abs(frontRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(frontRight.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
            &&((Math.abs(backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(backLeft.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband))
            &&((Math.abs(backRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(Math.abs(backRight.getAbsoluteEncoderRad()) - Math.PI) < OperationConstants.kSwerveDeadband)))           
        {
            System.out.println("CALIBRATED");
            stopModules();
            return true;
        }

        return false;
    }
}
//         if((Math.abs(frontLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) || (Math.abs(frontLeft.getAbsoluteEncoderRad()) > -OperationConstants.kSwerveDeadband))
//         {
//             frontLeft.stop();
//         }
//         if(((frontRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (frontRight.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//         {
//             frontRight.stop();
//         }
//         if(((backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backLeft.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//         {
//             backLeft.stop();
//         }
//         if(((backRight.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backRight.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//         {
//             backRight.stop();
//         }

//         if(
//             (((backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backLeft.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//             &&(((backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backLeft.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//             &&(((backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backLeft.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//             &&(((backLeft.getAbsoluteEncoderRad()) < OperationConstants.kSwerveDeadband) && (backLeft.getAbsoluteEncoderRad() > -OperationConstants.kSwerveDeadband))
//            )
//         {
//             System.out.println("CALIBRATED");
//             stopModules();
//             return true;
//         }

//         return false;
//     }
//     // public boolean modulesarezero()
//     // {        
//     //     if(frontLeft.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && frontLeft.getAbsoluteEncoderRad() >= 0)
//     //     {
//     //         frontLeft.stop();
//     //     }
//     //     if(frontRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && frontRight.getAbsoluteEncoderRad() >= 0)
//     //     {
//     //         frontRight.stop();
//     //     }
//     //     if(backLeft.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backLeft.getAbsoluteEncoderRad() >= 0)
//     //     {
//     //         backLeft.stop();
//     //     }
//     //     if(backRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backRight.getAbsoluteEncoderRad() >= 0)
//     //     {
//     //         backRight.stop();
//     //     }

//     //     if(
//     //         ((backRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backRight.getAbsoluteEncoderRad() >= 0))
//     //         &&((backRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backRight.getAbsoluteEncoderRad() >= 0))
//     //         &&((backRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backRight.getAbsoluteEncoderRad() >= 0))
//     //         &&((backRight.getAbsoluteEncoderRad() < OperationConstants.kSwerveDeadband && backRight.getAbsoluteEncoderRad() >= 0))
//     //     //    && Math.abs(frontRight.getAbsoluteEncoderRad()) < .2
//     //     //    &&Math.abs(backLeft.getAbsoluteEncoderRad()) < .2
//     //     //    &&Math.abs(backRight.getAbsoluteEncoderRad()) < .2
//     //        )
        
//     //     {
//     //         System.out.println("CALIBRATED");
//     //         stopModules();
//     //         return true;
//     //     }

//     //     return false;
//     // }
// }