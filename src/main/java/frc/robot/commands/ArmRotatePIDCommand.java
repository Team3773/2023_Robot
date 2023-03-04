package frc.robot.commands;

import frc.robot.subsystems.ArmRotateSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRotatePIDCommand extends CommandBase{
    private final ArmRotateSubsystem armRotateSub;
    private final PIDController pidController;

    public ArmRotatePIDCommand(ArmRotateSubsystem subsystem, double setpoint)
    {
        armRotateSub = subsystem;
        this.pidController = new PIDController(0.15, 0, 0); 
        pidController.setSetpoint(setpoint);
        
        addRequirements(armRotateSub);
    }
    @Override
    public void initialize() {
        pidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pidController.calculate(armRotateSub.getEncoderMeters());
        armRotateSub.setArmRotateSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armRotateSub.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}