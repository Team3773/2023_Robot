package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawPIDCommand extends CommandBase{
    private final ClawSubsystem clawSubsystem;
    private final PIDController pidController;

    public ClawPIDCommand(ClawSubsystem subsystem, double setpoint)
    {
        clawSubsystem = subsystem;
        this.pidController = new PIDController(0.15, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(clawSubsystem);
    }
    @Override
    public void initialize() {
        pidController.reset();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // MAY NEED DOULBE ARG METHOD HERE? NOT SURE IF .setSetpoint is right
        double speed = pidController.calculate(clawSubsystem.getEncoderMeters());
        clawSubsystem.setClawSpeed(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}