package frc.robot.commands;

import frc.robot.subsystems.ArmExtendSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmExtendCommand extends CommandBase{
    private final ArmExtendSubsystem armExtendSub;
    private double extendSpeed;

    public ArmExtendCommand(ArmExtendSubsystem subsystem, double extendSpeed)
    {
        armExtendSub = subsystem;
        this.extendSpeed = extendSpeed; 
        
        addRequirements(armExtendSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(extendSpeed) < 0.05)
        {
            extendSpeed = 0;
        }
        armExtendSub.setArmExtendSpeed(extendSpeed);
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