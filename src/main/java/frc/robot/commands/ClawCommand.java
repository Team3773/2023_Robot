package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase{
    private final ClawSubsystem clawSub;
    private double closeSpeed;
    private double openSpeed;

    public ClawCommand(ClawSubsystem subsystem, double closeSpeed, double openSpeed)
    {
        clawSub = subsystem; 
        this.closeSpeed = closeSpeed;
        this.openSpeed = openSpeed;
        
        addRequirements(clawSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double clawSpeed;

        if(Math.abs(closeSpeed) < 0.05)
        {
            clawSpeed = -openSpeed;
        }
        else if(Math.abs(openSpeed) < 0.05)
        {
            clawSpeed = closeSpeed; 
        }
        else
        {
            clawSpeed = 0;
        }

        clawSub.setClawSpeed(clawSpeed);
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