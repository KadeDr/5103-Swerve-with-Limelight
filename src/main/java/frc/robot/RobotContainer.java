// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.limelight.LocateAprilTagCommand;
import frc.robot.commands.shooter.CalculateTurretPosition;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final IntakeSubsystem m_robotIntake = new IntakeSubsystem(11);
        private final ShooterSubsystem m_robotShooter = new ShooterSubsystem(10, 9);

        // Controller
        private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        private final XboxController m_turretController = new XboxController(OIConstants.kTurretControllerPort);

        // Slew rate limiters for smooth joystick input (same as your original
        // Robot.java)
        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

        // Accurate period calculation using Timer
        private double m_currentPeriod = 0.02;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureButtonBindings();
                m_robotDrive.zeroHeading();

                // Default command: manual field-relative swerve drive with slew limiting and
                // real period
                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> {
                                        // Get TV (Exists) and TX (Angle)

                                        // Rumble ONLY if we see target AND are aimed within 3 degrees
                                        // if (hasTarget && Math.abs(tx) < 3.0) {
                                        //         // Check for CommandXboxController vs XboxController here
                                        //         m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
                                        // } else {
                                        //         m_driverController.setRumble(RumbleType.kBothRumble, 0);
                                        // }
                                        // Apply deadband and slew limiting
                                        double xInput = MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband);
                                        double yInput = MathUtil.applyDeadband(-m_driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband);
                                        double rotInput = MathUtil.applyDeadband(m_driverController.getRightX(),
                                                        OIConstants.kDriveDeadband);

                                        double xSpeed = -m_xspeedLimiter.calculate(xInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double ySpeed = -m_yspeedLimiter.calculate(yInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double rot = -m_rotLimiter.calculate(rotInput)
                                                        * DriveConstants.kMaxAngularSpeed;

                                        m_robotDrive.drive(xSpeed, ySpeed, rot, true, m_currentPeriod);
                                }, m_robotDrive));
        }

        private void configureButtonBindings() {
                // R1 (right bumper) → X formation (lock wheels)
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                // X button on driver controller → follow PathPlanner "Straight Line" path
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(Commands.defer(() -> {
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile("Straight Line");
                        return AutoBuilder.followPath(path);
                } catch (Exception e) {
                        DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                }
        }, java.util.Set.of(m_robotDrive)));

                // new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                //   .whileTrue(new AimAssistCommand(m_driverController, 1 , "limelight-main", m_xspeedLimiter, m_yspeedLimiter, m_rotLimiter, m_robotDrive));

                // new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(new FollowAlgaeCommand("algae", "limelight-main", m_robotDrive));
                new JoystickButton(m_driverController, XboxController.Button.kY.value).toggleOnTrue(new IntakeCommand(m_robotIntake, -2000)); // Intake fuel
                new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new IntakeCommand(m_robotIntake, 2000)); // Outtake fuel
                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).toggleOnTrue(new ShootCommand(m_robotShooter, m_robotShooter.getRPM())); // Toggle shooter
                new JoystickButton(m_turretController, XboxController.Button.kX.value).toggleOnTrue(new ConditionalCommand(new CalculateTurretPosition( // Toggle turntable
                        m_robotShooter, "limelight-main"),
                        new LocateAprilTagCommand(m_robotShooter, "limelight-main"),
                        () -> LimelightHelpers.getTV("limelight-main")).repeatedly());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return null;
                // Trajectory config
                // TrajectoryConfig config = new TrajectoryConfig(
                //                 AutoConstants.kMaxSpeedMetersPerSecond,
                //                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                //                 .setKinematics(DriveConstants.kDriveKinematics);

                // // Trajectory 1
                // Trajectory forwardRotation = TrajectoryGenerator.generateTrajectory(
                //                 new Pose2d(0, 0, new Rotation2d(0)),
                //                 List.of(
                //                                 new Translation2d(Units.feetToMeters(0.3), 0),
                //                                 new Translation2d(Units.feetToMeters(0.6), 0)),
                //                 new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(180)),
                //                 config);

                // var thetaController1 = new ProfiledPIDController(
                //                 AutoConstants.kPThetaController, 0, 0.5,
                //                 AutoConstants.kThetaControllerConstraints);
                // thetaController1.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                //                 forwardRotation,
                //                 m_robotDrive::getPose,
                //                 DriveConstants.kDriveKinematics,
                //                 new PIDController(AutoConstants.kPXController, 0, 0),
                //                 new PIDController(AutoConstants.kPYController, 0, 0),
                //                 thetaController1,
                //                 m_robotDrive::setModuleStates,
                //                 m_robotDrive);

                // return new SequentialCommandGroup(
                //                 // Reset odometry at start
                //                 new InstantCommand(() -> m_robotDrive.resetOdometry(forwardRotation.getInitialPose()),
                //                                 m_robotDrive),

                //                 // Run trajectory 1
                //                 swerveControllerCommand1,

                //                 // Lock wheels in X formation at end
                //                 new InstantCommand(() -> m_robotDrive.setModuleStates(new SwerveModuleState[] {
                //                                 new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                //                                 new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                //                                 new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                //                                 new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                //                 }), m_robotDrive));

                // Your commented-out longer auto is preserved below for reference
                /*
                 * return new SequentialCommandGroup(
                 * // Reset odometry at start
                 * new InstantCommand(() ->
                 * m_robotDrive.resetOdometry(forwardRotation.getInitialPose()), m_robotDrive),
                 * 
                 * // Trajectory 1
                 * swerveControllerCommand1,
                 * 
                 * // Immediately zero drive after traj1
                 * new InstantCommand(() -> m_robotDrive.setModuleStates(new SwerveModuleState[]
                 * {
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                 * new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                 * }), m_robotDrive));
                 */
        }
}