// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperationConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceOnBeamCommand extends CommandBase {
  private final SwerveSubsystem swerveSub;
  private double setPoint;
  private final PIDController pidController;

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

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 1, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSub.setModuleStates(moduleStates);
    // double error = OperationConstants.kBeam_Balance_Goal_Degrees - currentAngle;
    // double drivePower = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);

    // // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    // if (drivePower < 0) {
    //   drivePower *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    // }

    // // Limit the max power
    // if (Math.abs(drivePower) > 0.4) {
    //   drivePower = Math.copySign(0.4, drivePower);
    // }

    // m_DriveSubsystem.drive(drivePower, drivePower);
    
    // // Debugging Print Statments
    // System.out.println("Current Angle: " + currentAngle);
    // System.out.println("Error " + error);
    // System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
