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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRotateCommand;
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
    private JoystickButton driverButtonA;

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getLeftY(), //left y
                () -> driverJoytick.getLeftX(), //left x
                () -> driverJoytick.getRightX(), // right x
                () -> !driverJoytick.getLeftBumper()));

          clawSubsystem.setDefaultCommand(new ClawCommand(clawSubsystem, operatorJoystick.getRightTriggerAxis(), operatorJoystick.getLeftTriggerAxis()));
          armExtendSubsystem.setDefaultCommand(new ArmExtendCommand(armExtendSubsystem, operatorJoystick.getRightY()));
          armRotateSubsystem.setDefaultCommand(new ArmRotateCommand(armRotateSubsystem, operatorJoystick.getLeftY()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        buttonY = new JoystickButton(operatorJoystick, 4);
        buttonA = new JoystickButton(operatorJoystick, 1);

        driverButtonA = new JoystickButton(driverJoytick, 1);

        buttonY.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(1), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));
        buttonA.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(-1), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));

        driverButtonA.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
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
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        // Other points
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                //Final points
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
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
                new InstantCommand(() -> armExtendSubsystem.setArmExtendSpeed(0.8)),
                // Initialize swerve
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                // Follow swerve trajectory
                swerveControllerCommand,
                // Stop swerve
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
