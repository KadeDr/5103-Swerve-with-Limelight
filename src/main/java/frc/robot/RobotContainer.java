// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.AutonomousShootCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.limelight.LocateAprilTagCommand;
import frc.robot.commands.shooter.CalculateTurretPosition;
import frc.robot.commands.shooter.RotateCommand;
import frc.robot.commands.shooter.ShootWithLimelightCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final IntakeSubsystem m_robotIntake = new IntakeSubsystem(IntakeConstants.canId,
                        IntakeConstants.leftCanId);
        private final ShootSubsystem m_robotShooter = new ShootSubsystem(ShooterConstants.shooterCanId);
        private final TurretSubsystem m_robotTurret = new TurretSubsystem(ShooterConstants.turntableCanId);
        private final IndexerSubsystem m_robotIndexer = new IndexerSubsystem(IndexerConstants.canId);
        // private final ClimbSubsystem m_climb = new ClimbSubsystem(14);

        // Controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        @SuppressWarnings("unused")
        private final CommandXboxController m_turretController = new CommandXboxController(
                        OIConstants.kTurretControllerPort);
        // A dedicated controller for testing, plugged into port 2
        private final CommandXboxController m_testController = new CommandXboxController(2);

        // Slew rate limiters for smooth joystick input (same as your original
        // Robot.java)
        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

        private boolean fieldRelative = true;

        // Accurate period calculation using Timer
        private double m_currentPeriod = 0.02;

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("Shoot", new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 1000));
                // new ParallelCommandGroup(
                //         new ShootWithLimelightCommand(m_robotShooter, m_robotIndexer, 12, "limelight-main", m_driverController),
                //         new CalculateTurretPosition(m_robotTurret, "limelight-main")
                // ));

                configureButtonBindings();
                m_robotDrive.zeroHeading();

                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> {
                                        // Get TV (Exists) and TX (Angle)

                                        // Rumble ONLY if we see target AND are aimed within 3 degrees
                                        // if (hasTarget && Math.abs(tx) < 3.0) {
                                        // // Check for CommandXboxController vs XboxController here
                                        // m_driverController.setRumble(RumbleType.kBothRumble, 0.5);
                                        // } else {
                                        // m_driverController.setRumble(RumbleType.kBothRumble, 0);
                                        // }
                                        // Apply deadband and slew limiting
                                        double xInput = MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband);
                                        double yInput = MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband);
                                        double rotInput = MathUtil.applyDeadband(m_driverController.getRightX(),
                                                        OIConstants.kDriveDeadband);

                                        double xSpeed = -m_xspeedLimiter.calculate(xInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double ySpeed = -m_yspeedLimiter.calculate(yInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double rot = -m_rotLimiter.calculate(rotInput)
                                                        * DriveConstants.kMaxAngularSpeed;

                                        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, m_currentPeriod);
                                }, m_robotDrive));
                
                Command eightAuto = new PathPlannerAuto("8 Shoot");
                
                m_chooser.setDefaultOption("8 Shoot", eightAuto);
                m_chooser.addOption("Do Nothing", new WaitCommand(20));

                SmartDashboard.putData("Auto Mode", m_chooser);
        }

        private void configureButtonBindings() {
                // R1 (right bumper) → X formation (lock wheels)
                // m_driverController.rightBumper()
                // .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

                // X button on driver controller → follow PathPlanner "Straight Line" path
                // m_driverController.x()
                // .whileTrue(Commands.defer(() -> {
                // try {
                // PathPlannerPath path = PathPlannerPath.fromPathFile("Straight Line");
                // m_robotDrive.resetOdometry(path.getStartingHolonomicPose().get());
                // return AutoBuilder.followPath(path);
                // } catch (Exception e) {
                // DriverStation.reportError("Failed to load path: " + e.getMessage(),
                // e.getStackTrace());
                // return Commands.none();
                // }
                // }, java.util.Set.of(m_robotDrive)));
                // m_driverController.x()
                //                 .whileTrue(
                //                                 Commands.defer(() -> {
                //                                         try {
                //                                                 // Load the full auto routine (must match the exact name
                //                                                 // of the .auto file)
                //                                                 return new PathPlannerAuto(
                //                                                                 "8 Shoot");
                //                                         } catch (Exception e) {
                //                                                 DriverStation.reportError(
                //                                                                 "Failed to load auto: "
                //                                                                                 + e.getMessage(),
                //                                                                 e.getStackTrace());
                //                                                 return Commands.none();
                //                                         }
                //                                 }, java.util.Set.of(m_robotDrive)));
                // ;

                m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.ResetGyro()));
                m_driverController.y().toggleOnTrue(new IntakeCommand(m_robotIntake, -1500)); // Intake fuel
                m_driverController.a().whileTrue(new IntakeCommand(m_robotIntake, 750)); // Outtake fuel
                m_driverController.leftBumper().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
                // m_driverController.povLeft().toggleOnTrue(new
                // SpinIndexerCommand(m_robotIndexer, 1500));
                // m_driverController.rightBumper().toggleOnTrue(new
                // ShootCommand(m_robotShooter, 2500)); // Toggle shooter

                // m_driverController.rightBumper()
                //                 .toggleOnTrue(new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 3300)); // Toggle
                // shooter
                m_driverController.rightBumper().toggleOnTrue(
                new ShootWithLimelightCommand(m_robotShooter, m_robotIndexer, 12,
                "limelight-main", m_driverController)); // Toggle
                // shooter

                // m_turretController.leftTrigger().whileTrue(new RotateCommand(m_robotTurret, m_turretController));
                // m_turretController.rightTrigger().whileTrue(new RotateCommand(m_robotTurret, m_turretController));
                m_turretController.a().onTrue(new InstantCommand(() -> m_robotTurret.ResetEncoder()));
                m_turretController.x().toggleOnTrue(new CalculateTurretPosition(m_robotTurret, "limelight-main"));
                // m_turretController.x()
                // .toggleOnTrue(new ConditionalCommand(
                // new CalculateTurretPosition(m_robotTurret, "limelight-main"),
                // new LocateAprilTagCommand(m_robotShooter, m_robotTurret, "limelight-main"),
                // () -> LimelightHelpers.getTV("limelight-main")).repeatedly());
                // m_turretController.x().whileTrue(new ClimbCommand(m_climb, 1));
                // m_turretController.b().whileTrue(new ClimbCommand(m_climb, -1));
        }

        @Override
        public void periodic() {
                SmartDashboard.putBoolean("Field Relative", fieldRelative);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}