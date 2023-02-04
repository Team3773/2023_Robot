package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperationConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase{
    
    public ArmRotateSubsystem() {
      }
      CANSparkMax armRotateMotor = new CANSparkMax(OperationConstants.armRotateMotorChannel, MotorType.kBrushless);
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmRotateSpeed(double speed)
      {
        armRotateMotor.set(speed);
      }

      public void stopMotor()
      {
        armRotateMotor.set(0);
      }
}