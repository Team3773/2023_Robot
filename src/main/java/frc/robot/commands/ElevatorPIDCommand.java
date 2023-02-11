package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorPIDCommand extends CommandBase{
    private final ElevatorSubsystem elevatorSub;
    private final PIDController pidController;

    public ElevatorPIDCommand(ElevatorSubsystem subsystem, double setpoint)
    {
        elevatorSub = subsystem;
        this.pidController = new PIDController(0, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(elevatorSub);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pidController.calculate(elevatorSub.getEncoderMeters());
        elevatorSub.setElevatorSpeed(speed);

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