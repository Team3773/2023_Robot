package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase{
    
    public ArmRotateSubsystem() {
      }
      CANSparkMax armRotateMotor = new CANSparkMax(OperationConstants.armRotateMotorChannel, MotorType.kBrushless);
      RelativeEncoder armRotateEncoder = armRotateMotor.getEncoder();
      DigitalInput limitSwitch = new DigitalInput(1);
    
      @Override
      public void periodic() {
        armRotateEncoder.setPositionConversionFactor(OperationConstants.kArmRotateEncoderRot2Meter);
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmRotateSpeed(double speed)
      {
        if(!limitSwitch.get())
        {
          armRotateMotor.set(speed);
        }
        else if(speed > 0)
        {
          armRotateMotor.set(speed);
        }
        else
        {
          armRotateMotor.stopMotor();
        }
      }

      public void stopMotor()
      {
        armRotateMotor.set(0);
      }

      public double getEncoderMeters() {
        return armRotateEncoder.getPosition();
      }
}