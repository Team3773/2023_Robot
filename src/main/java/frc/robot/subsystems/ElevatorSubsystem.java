package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    CANSparkMax elevatorMotor = new CANSparkMax(OperationConstants.elevatorMotorChannel, MotorType.kBrushless);
    RelativeEncoder elvatorEncoder = elevatorMotor.getEncoder();

    public ElevatorSubsystem() {
      }
    
      @Override
      public void periodic() {
        elvatorEncoder.setPositionConversionFactor(OperationConstants.kElevatorEncoderRot2Meter);
        SmartDashboard.putNumber("Elevator Encoder: ", getEncoderMeters());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }

      public void setElevatorSpeed(double speed)
      {
        double error;
        double outputMultiplier;
        
        if(elvatorEncoder.getPosition() < OperationConstants.kElevatorMiddlePosition)
        {
          error = OperationConstants.kElevatorBottomPosition - elvatorEncoder.getPosition();
          outputMultiplier = error / OperationConstants.kElevatorTopPosition;
        }
        else
        {
          error = OperationConstants.kElevatorTopPosition - elvatorEncoder.getPosition();
          outputMultiplier = error / OperationConstants.kElevatorTopPosition;
        }

        elevatorMotor.set(speed * outputMultiplier);
      }
      public void stopMotor()
      {
        elevatorMotor.set(0);
      }

      public double getEncoderMeters() {
        return elvatorEncoder.getPosition();
      }
}