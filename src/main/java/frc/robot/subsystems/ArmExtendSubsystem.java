package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperationConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase{
    
    public ArmExtendSubsystem() {
      }
      CANSparkMax armExtendMotor = new CANSparkMax(OperationConstants.armExtendMotorChannel, MotorType.kBrushless);
      RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();
    
      @Override
      public void periodic() {
        armExtendEncoder.setPositionConversionFactor(OperationConstants.kArmExtendEncoderRot2Meter);
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
        return armExtendEncoder.getPosition();
      }
}