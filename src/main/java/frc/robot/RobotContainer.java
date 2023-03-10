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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperationConstants;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ClawPIDCommand;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.CalibrateWheelsCommand;
import frc.robot.commands.ElevatorPIDCommand;
import frc.robot.commands.ArmExtendPIDCommand;

import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmExtendSubsystem;
import frc.robot.subsystems.ArmRotateSubsystem;
import frc.robot.subsystems.SwerveModule;



public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ArmExtendSubsystem armExtendSubsystem = new ArmExtendSubsystem();
    private final ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();
//     private final SwerveModule swerveModule = new SwerveModule();

     // OPERATOR BUTTONS
    private JoystickButton buttonA;
    private JoystickButton buttonB;
    private JoystickButton buttonX;
    private JoystickButton buttonY;
    private JoystickButton buttonLeftTrigger;
    private JoystickButton buttonRightTrigger;

    // DRIVER BUTTONS
    private JoystickButton driverButtonA;
    private JoystickButton driverButtonB;
    private JoystickButton driverButtonX;
    private JoystickButton driverButtonY;
//     private JoystickButton driverButtonLeftTrigger;
    private JoystickButton driverButtonRightTrigger;

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController operatorJoystick = new XboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    private double operateLeftYSpeed = 0;
    private double operateRightYSpeed = 0;

    public RobotContainer() {
        m_chooser.setDefaultOption("Simple Auto", blueAutoCommand());
        m_chooser.addOption("Complex Auto", redAutoCommand());
        m_chooser.addOption("Test Command", testCommand());


        // if(Math.abs(operatorJoystick.getLeftY()) > .1)
        // {
        //         operateLeftYSpeed = operatorJoystick.getLeftY();
        // }

        // if(Math.abs(operatorJoystick.getRightY()) > .1)
        // {
        //         operateRightYSpeed = operatorJoystick.getRightY();
        // }

        Shuffleboard.getTab("Autonomous ").add(m_chooser);

        /*
         * LEFT Y: X-AXIS
         * LEFT X: Y-AXIS
         * RIGHT X = ROTATION
         * LEFT BUMPER = FIELD ORIENTED TRUE OR  NOT
         */
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getLeftY(),
                () -> driverJoytick.getLeftX(),
                () -> driverJoytick.getRightX(),
                () -> !driverJoytick.getLeftBumper())); // Try changing to true or false
        
        // Open claw with right trigger axis. Close claw with left trigger axis. 
        clawSubsystem.setDefaultCommand(new ClawCommand(clawSubsystem, () -> operatorJoystick.getRightTriggerAxis() * 0.1, () -> operatorJoystick.getLeftTriggerAxis() * 0.1));

        // Extend or retract arm with right y-axis.
        armExtendSubsystem.setDefaultCommand(new ArmExtendCommand(armExtendSubsystem, () -> operateLeftYSpeed));

        // Rotate arm with left y-axis.
        armRotateSubsystem.setDefaultCommand(new ArmRotateCommand(armRotateSubsystem, () -> operateRightYSpeed));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // OPERATOR BUTTONS
        buttonA = new JoystickButton(operatorJoystick, 1);
        buttonB = new JoystickButton(operatorJoystick, 2);
        buttonX = new JoystickButton(operatorJoystick, 3);
        buttonY = new JoystickButton(operatorJoystick, 4);
        buttonLeftTrigger = new JoystickButton(operatorJoystick, 5);
        buttonRightTrigger = new JoystickButton(operatorJoystick, 6);

        // DRIVER BUTTONS
        driverButtonA = new JoystickButton(driverJoytick, 1);
        driverButtonB = new JoystickButton(driverJoytick, 2);
        driverButtonX = new JoystickButton(driverJoytick, 3);
        driverButtonY = new JoystickButton(driverJoytick, 4);
        // driverButtonLeftTrigger = new JoystickButton(driverJoytick, 5);
        driverButtonRightTrigger = new JoystickButton(driverJoytick, 6);

        // MANUALLY ELEVATE
        buttonY.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(.15), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));
        buttonA.whileTrue(new StartEndCommand(() -> elevatorSubsystem.setElevatorSpeed(-.15), () -> elevatorSubsystem.stopMotor(), elevatorSubsystem));

        // PLACE TOP
        buttonB.onTrue(new SequentialCommandGroup(
                new ElevatorPIDCommand(elevatorSubsystem, 0),
                new ArmExtendPIDCommand(armExtendSubsystem, 51),
                new ClawPIDCommand(clawSubsystem, -50)
        ));

        // PLACE BOTTOM
        buttonX.onTrue(new SequentialCommandGroup(
                new ElevatorPIDCommand(elevatorSubsystem, 0),
                new ArmExtendPIDCommand(armExtendSubsystem, 116)
                ));
        
        // PICK UP FROM FLOOR
        buttonRightTrigger.onTrue(new ElevatorPIDCommand(elevatorSubsystem, 0));

        /*========================DRIVER======================================*/

        // ZERO GYRO
        driverButtonA.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

        // ZERO ENCODERS FOR OPERATIONS
        driverButtonB.onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> elevatorSubsystem.zeroEncoder()),
                new InstantCommand(() -> armExtendSubsystem.zeroEncoder()),
                new InstantCommand(() -> clawSubsystem.zeroEncoder()),
                new InstantCommand(() -> armRotateSubsystem.zeroEncoder())
                ));
        
        // CALIBRATE WHEELS
        driverButtonX.onTrue(new CalibrateWheelsCommand(swerveSubsystem));

        driverButtonY.onTrue(new InstantCommand(() -> swerveSubsystem.syncEncoders()));
        
        // BALANCE IN TELEOP
        driverButtonRightTrigger.onTrue(new BalanceOnBeamCommand(swerveSubsystem, OperationConstants.kBeam_Balance_Goal_Degrees));
    }

    public Command blueAutoCommand()
    {
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
                        new Translation2d(2.90, 2.71),
                        new Translation2d(4.90, 2.71)),
                //Final points
                new Pose2d(6.40, 2.71, Rotation2d.fromDegrees(180)), // CHANGED FROM 180 deg
                trajectoryConfig);
        
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
        SequentialCommandGroup firstSequentialCommandGroup = new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                // Follow swerve trajectory defined in 2
                swerveControllerCommand,
                // Balance on Beam
                // // Stop swerve
                new InstantCommand(() -> swerveSubsystem.stopModules()));
        
        return firstSequentialCommandGroup;
        } 

    public Command redAutoCommand()
    {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // Alternate trajectory
        Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
                // Initial point
                new Pose2d(14.67, 2.71, new Rotation2d(0)),
                List.of(
                        // Other points
                        new Translation2d(13.1, 2.71),
                        new Translation2d(1.8, 2.71)),
                //Final points
                new Pose2d(10.620, 2.71, Rotation2d.fromDegrees(0)), // CHANGED FROM 180 deg
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // Profile to max interval
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand secondSwerveControllerCommand = new SwerveControllerCommand(
                secondTrajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
                
        SequentialCommandGroup secondSequentialCommandGroup = new SequentialCommandGroup(
                
                new InstantCommand(() -> swerveSubsystem.resetOdometry(secondTrajectory.getInitialPose())),
                // Follow swerve trajectory defined in 2
                secondSwerveControllerCommand,
                // Balance on Beam
                // Stop swerve
                new InstantCommand(() -> swerveSubsystem.stopModules()));
        return secondSequentialCommandGroup;
    }

    public Command testCommand()
    {          
        SequentialCommandGroup testSequentialCommandGroup = new SequentialCommandGroup(
                new ArmExtendPIDCommand(armExtendSubsystem, 51),
                new ClawPIDCommand(clawSubsystem, -50)
                );
        return testSequentialCommandGroup;
    }
    
    public Command getAutonomousCommand() {
        // return redAutoCommand();
        return m_chooser.getSelected();
    }
}