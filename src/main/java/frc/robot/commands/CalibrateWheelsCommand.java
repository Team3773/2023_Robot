// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class CalibrateWheelsCommand extends CommandBase {
  private final SwerveSubsystem swerveSub;

  /** Creates a new CalibrateWheelsCommand. */
  public CalibrateWheelsCommand(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;
    addRequirements(swerveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSub.resetStates();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(swerveSub.modulesarezero())
    {
      swerveSub.zeroAbsEncodersSwerveSub();
      return true;
    }
    return false;
  }
}