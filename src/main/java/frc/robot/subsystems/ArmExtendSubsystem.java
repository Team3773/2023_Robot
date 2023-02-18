package frc.robot.subsystems;

import frc.robot.Constants.OperationConstants;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    
    public ArmExtendSubsystem() {
      }
      WPI_TalonSRX armExtendMotor = new WPI_TalonSRX(OperationConstants.armExtendMotorChannel);
      Encoder encoder = new Encoder(OperationConstants.karmExtendEncoderA, OperationConstants.karmExtendEncoderB);
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmExtendSpeed(double speed)
      {
        armExtendMotor.set(speed);
      }

      public void stopMotor()
      {
        armExtendMotor.set(0);
      }
      public double getEncoderMeters() {
        return encoder.get() * OperationConstants.kArmExtendEncoderRot2Meter;
      }
}