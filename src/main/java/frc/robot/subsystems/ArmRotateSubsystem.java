package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase{
    
    public ArmRotateSubsystem() {
      }
      WPI_TalonSRX armRotateMotor = new WPI_TalonSRX(OperationConstants.armExtendMotorChannel);
      Encoder armRotatEncoder = new Encoder(OperationConstants.karmExtendEncoderA, OperationConstants.karmExtendEncoderB);
      DigitalInput limitSwitch = new DigitalInput(1);
    
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
        return armRotatEncoder.get() * OperationConstants.kArmRotateEncoderRot2Meter;
      }
}