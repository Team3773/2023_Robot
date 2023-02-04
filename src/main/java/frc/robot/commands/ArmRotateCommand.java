package frc.robot.commands;

import frc.robot.subsystems.ArmRotateSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRotateCommand extends CommandBase{
    private final ArmRotateSubsystem armRotateSub;
    private double rotateSpeed;

    public ArmRotateCommand(ArmRotateSubsystem subsystem, double rotateSpeed)
    {
        armRotateSub = subsystem; 
        
        addRequirements(armRotateSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
        return false;
    }
}