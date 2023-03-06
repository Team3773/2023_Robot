package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperationConstants;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ClawPIDCommand;
import frc.robot.commands.ArmRotatePIDCommand;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.ArmExtendPIDCommand;
import frc.robot.commands.BalanceOnBeamCommand;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ArmRotateSubsystem;



public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ArmExtendSubsystem armExtendSubsystem = new ArmExtendSubsystem();
    private final ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();

    /*
     * Xbox Controller A for Swerve. Xbox Controller B for operators. 
     * Joysticks: swerve.
     * Arm extend: RightY
     * Arm rotate: LeftY
     * Elevator: Y: Up | A: Down
     * Claw: Left Trigger: Open | Right Trigger: Close 
     */

    private JoystickButton buttonY;
    private JoystickButton buttonA;
    private JoystickButton buttonX;
    private JoystickButton buttonB;
    private JoystickButton driverButtonA;
    private JoystickButton buttonRightTrigger;
    private JoystickButton buttonLeftTrigger;

    private JoystickButton driverButtonB;
    private JoystickButton driverButtonY;

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getLeftY(), //left y
                () -> driverJoytick.getLeftX(), //left x
                () -> driverJoytick.getRightX(), // right x
                () -> !driverJoytick.getLeftBumper()));

        // USB 1. BLACK XBOX
        
        // Open claw with right trigger axis. 
        // Close claw with left trigger axis. 
        clawSubsystem.setDefaultCommand(new ClawCommand(clawSubsystem, () -> operatorJoystick.getRightTriggerAxis() * 0.1, () -> operatorJoystick.getLeftTriggerAxis() * 0.1));
        // Extend or retract arm with right y-axis.
        armExtendSubsystem.setDefaultCommand(new ArmExtendCommand(armExtendSubsystem, () -> operatorJoystick.getRightY()));
        // Rotate arm with left y-axis.
        armRotateSubsystem.setDefaultCommand(new ArmRotateCommand(armRotateSubsystem, () -> operatorJoystick.getLeftY()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // USB 1. NOT CLEAR XBOX
        buttonY = new JoystickButton(operatorJoystick, 4);
        buttonA = new JoystickButton(operatorJoystick, 1);

        buttonB = new JoystickButton(operatorJoystick, 2);
        buttonX = new JoystickButton(operatorJoystick, 3);
        buttonRightTrigger = new JoystickButton(operatorJoystick, 5);
        buttonLeftTrigger = new JoystickButton(operatorJoystick, 6);

        driverButtonA = new JoystickButton(driverJoytick, 1);
        driverButtonB = new JoystickButton(driverJoytick, 2);
        driverButtonY = new JoystickButton(driverJoytick, 4);

        // buttonY.whileTrue(new ElevatorPIDCommand(elevatorSubsystem, 0));
        buttonY.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(.15), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));
        buttonA.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(-.15), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));

        // PLACE TOP
        buttonB.onTrue(new ElevatorPIDCommand(elevatorSubsystem, -20));

        // PLACE BOTTOM
        buttonX.onTrue(new ElevatorPIDCommand(elevatorSubsystem, -35));
        
        // PICK UP FROM FLOOR
        buttonRightTrigger.onTrue(new ElevatorPIDCommand(elevatorSubsystem, -100));

        // buttonRightTrigger.onTrue(new ArmExtendPIDCommand(armExtendSubsystem, 118));
        // buttonLeftTrigger.onTrue(new ArmRotatePIDCommand(armRotateSubsystem, 55));

        // USB 0. CLEAR XBOX
        driverButtonA.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

        // Balance in teleop
        driverButtonB.onTrue(new BalanceOnBeamCommand(swerveSubsystem, OperationConstants.kBeam_Balance_Goal_Degrees));

        driverButtonY.onTrue(new InstantCommand(() -> swerveSubsystem.resetStates()));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Initial point
                new Pose2d(1.96, 2.71, new Rotation2d(0)),
                List.of(
                        // Other points
                        new Translation2d(2.98, 2.71),
                        new Translation2d(3.3, 2.71)),
                //Final points
                new Pose2d(3.90, 2.71, Rotation2d.fromDegrees(0)), // CHANGED FROM 180 deg
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // Profile to max interval
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                /**
                 * Path description: 
                 * Robot starts in community by gates. 
                 * Extends arm. 
                 * Opens claw. 
                 * Places cube.
                 * Drives back onto charge station.
                 */

                 // Extend Arm
                new ArmExtendPIDCommand(armExtendSubsystem, OperationConstants.kArmExtendSetpoint),
                // Open Claw
                new ClawPIDCommand(clawSubsystem, OperationConstants.kClawSetpoint),
                // Initialize swerve
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                // Follow swerve trajectory defined in 2
                swerveControllerCommand,
                // Balance on Beam
                new BalanceOnBeamCommand(swerveSubsystem, OperationConstants.kBeam_Balance_Goal_Degrees),
                // Stop swerve
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}