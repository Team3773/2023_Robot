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

    // Runs when command starts
    @Override
    public void initialize() {
    }

    // Runs repeatedly when command is called
    @Override
    public void execute() {
        if(Math.abs(extendSpeed) < 0.05)
        {
            extendSpeed = 0;
        }
        armExtendSub.setArmExtendSpeed(extendSpeed);
    }

    // Runs when command ends
    @Override
    public void end(boolean interrupted) {
    }

    // Returns when command is finished. Can also be interupted in container.
    @Override
    public boolean isFinished() {
        return false;
    }
}