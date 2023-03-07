// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceOnBeamCommand extends CommandBase {
  private final SwerveSubsystem swerveSub;
  private final PIDController pidController;
  private int balanceCounter = 0;

  /** Creates a new BalanceOnBeamCommand. */
  public BalanceOnBeamCommand(SwerveSubsystem swerveSub, double setPoint) {
    this.swerveSub = swerveSub;
    this.pidController = new PIDController(0.15, 0, 0); 

    pidController.setSetpoint(setPoint);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(swerveSub.getPitch());

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, speed, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSub.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(swerveSub.getPitch()) < 3){
      balanceCounter += 1;
    }else{
      balanceCounter = 0;
    }
    if(balanceCounter >= 20){
      return true;
    }
    return false;
  }
}
