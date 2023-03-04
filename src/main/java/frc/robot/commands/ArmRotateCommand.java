package frc.robot.commands;

import frc.robot.subsystems.ArmRotateSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRotateCommand extends CommandBase{
    private final ArmRotateSubsystem armRotateSub;
    private final Supplier<Double> rotateSpeedFunction;
    private int armRotateCounter = 0;

    public ArmRotateCommand(ArmRotateSubsystem subsystem, Supplier<Double> rotateSpeedFunction)
    {
        armRotateSub = subsystem;
        this.rotateSpeedFunction = rotateSpeedFunction; 
        
        addRequirements(armRotateSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rotateSpeed = rotateSpeedFunction.get();

        if(Math.abs(rotateSpeed) < 0.05)
        {
            rotateSpeed = 0;
        }

        armRotateSub.setArmRotateSpeed(rotateSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(Math.abs(armRotateSub.getEncoderMeters()) < 3){
            armRotateCounter += 1;
          }else{
            armRotateCounter = 0;
          }
          if(armRotateCounter >= 20){
            return true;
          }
          return false;
        }
}