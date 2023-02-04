package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(OperationConstants.elevatorMotorChannel);

    public ElevatorSubsystem() {
      }
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }

      public void setElevatorSpeed(double speed)
      {
        elevatorMotor.set(speed);
      }
      public void stopMotor()
      {
        elevatorMotor.set(0);
      }
}
