package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants.OperationConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase{
    
    public ArmRotateSubsystem() {
      }
      VictorSPX armRotateMotor = new VictorSPX(OperationConstants.armRotateMotorChannel);
      Encoder armRotatEncoder = new Encoder(OperationConstants.karmRotateEncoderA, OperationConstants.karmRotateEncoderB);
      DigitalInput limitSwitch = new DigitalInput(OperationConstants.limitSwitchPort);
    
      @Override
      public void periodic() {
        SmartDashboard.putNumber("Arm Rotate Encoder", getEncoderMeters());
        // This method will be called once per scheduler run
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
      public void setArmRotateSpeed(double speed)
      {
        // if(!limitSwitch.get())
        // {
        //   armRotateMotor.set(ControlMode.PercentOutput, speed);
        // }
        // else if(speed > 0)
        // {
        //   armRotateMotor.set(ControlMode.PercentOutput, speed);
        // }
        // else
        // {
        //   stopMotor();
        // }
        armRotateMotor.set(ControlMode.PercentOutput, speed);
      }

      public void stopMotor()
      {
        armRotateMotor.set(ControlMode.PercentOutput, 0);
      }

      public double getEncoderMeters() {
        return armRotatEncoder.get() * OperationConstants.kArmRotateEncoderRot2Meter;
      }
}