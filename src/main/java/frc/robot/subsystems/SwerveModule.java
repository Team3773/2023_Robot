package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); 
        
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // calibrateWheels(turningMotor, absoluteEncoder);

        resetEncoders();
    }

    // public void calibrateWheels()
    // {
    //     PIDController pid = new PIDController(0.5, 0, 0);
    //     turningMotor.set(pid.calculate(absoluteEncoder.getPosition(), 0.0));
    // }
    // public void calibrateWheels(CANSparkMax turnMotor, CANCoder absCanCoder)
    // {
    //     PIDController pid = new PIDController(0.5, 0.5, 0.5);
    //     turningMotor.set(pid.calculate(absoluteEncoder.getPosition(), 0.0));
    // }

    public void toSmartDashboard()
    {
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", Double.toString(getAbsoluteEncoderRad()));
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetModuleStates()
    {
        PIDController calibratePIDController = new PIDController(.05, 0, 0);
        turningMotor.set(calibratePIDController.calculate(getAbsoluteEncoderRad(), 0));
        // SmartDashboard.putString("PID number", Double.toString(calibratePIDController.calculate(getAbsoluteEncoderRad(), 0)));
        // System.out.println("RESETTTT!");
    }

    public double getAbsoluteEncoderRad() {
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180;        
        angle -= absoluteEncoderOffsetRad;
        angle *= (absoluteEncoderReversed ? -1.0 : 1.0);

        if(Math.abs(angle)> Math.PI){
            //need to wrap

            double wrapValue = Math.abs(angle) - Math.PI;
            if(angle > 0){
                return (-Math.PI + wrapValue);
            }else{
                return (Math.PI - wrapValue);
            }       
        }  

        return angle;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", Double.toString(getAbsoluteEncoderRad()));

    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
}