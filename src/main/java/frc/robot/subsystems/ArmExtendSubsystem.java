package frc.robot.subsystems;

import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    
    public ArmExtendSubsystem() {
      }
      TalonSRX armExtendMotor = new TalonSRX(OperationConstants.armExtendMotorChannel);
      Encoder encoder = new Encoder(OperationConstants.karmExtendEncoderA, OperationConstants.karmExtendEncoderB);
    
      @Override
      public void periodic() {
        SmartDashboard.putNumber("Arm Extend Encoder: ", encoder.getDistance());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmExtendSpeed(double speed)
      {
        armExtendMotor.set(ControlMode.PercentOutput, speed * OperationConstants.kArmExtendDampner);
      }

      public void stopMotor()
      {
        armExtendMotor.set(ControlMode.PercentOutput, 0);
      }

      public void setCurrentLimit()
      {
        armExtendMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
      }

      public double getEncoderMeters() {
        return encoder.get() * OperationConstants.kArmExtendEncoderRot2Meter;
      }
}