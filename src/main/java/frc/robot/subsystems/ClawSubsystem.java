package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperationConstants;

public class ClawSubsystem extends SubsystemBase{
    
    public ClawSubsystem() {
      }
      CANSparkMax clawMotor = new CANSparkMax(OperationConstants.clawMotorChannel, MotorType.kBrushless);
      RelativeEncoder clawEncoder = clawMotor.getEncoder();  
    
      @Override
      public void periodic() {
        clawEncoder.setPositionConversionFactor(OperationConstants.kClawEncoderRot2Meter);
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setClawSpeed(double speed)
      {
        clawMotor.set(speed);
      }

      public void stopMotor()
      {
        clawMotor.set(0);
      }

    public double getEncoderMeters() {
      return clawEncoder.getPosition();
    }
}
