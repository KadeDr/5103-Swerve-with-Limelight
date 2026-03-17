package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.AutonomousShootCommand;
import frc.robot.commands.intake.AutonomousIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.CalculateTurretPosition;
import frc.robot.commands.shooter.RotateCommand;
import frc.robot.commands.shooter.ShootWithLimelightCommand;
import frc.robot.commands.shooter.SpindexerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer extends SubsystemBase {
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final IntakeSubsystem m_robotIntake = new IntakeSubsystem(IntakeConstants.canId,
                        IntakeConstants.leftCanId);
        private final ShootSubsystem m_robotShooter = new ShootSubsystem(ShooterConstants.shooterCanId);
        private final TurretSubsystem m_robotTurret = new TurretSubsystem(ShooterConstants.turntableCanId);
        private final IndexerSubsystem m_robotIndexer = new IndexerSubsystem(IndexerConstants.canId);

        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController m_turretController = new CommandXboxController(
                        OIConstants.kTurretControllerPort);

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

        private boolean fieldRelative = true;

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<String> m_alliance = new SendableChooser<>();

        public RobotContainer() {
                NamedCommands.registerCommand("Shoot",
                                new ParallelCommandGroup(
                                                new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950,
                                                                m_driverController),
                                                new AutonomousIntakeCommand(m_robotIntake, -1500)));
                // new ParallelCommandGroup(
                // new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950,
                // m_driverController)));
                // new CalculateTurretPosition(m_robotTurret, "limelight-main",
                // m_alliance.getSelected())));

                configureButtonBindings();
                m_robotDrive.zeroHeading();

                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> {
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

                                        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, 0);
                                }, m_robotDrive));

                setupShuffleboard();
        }

        private void configureButtonBindings() {
                // ====================
                // Driver Controls (Stone)
                // ====================
                m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.ResetGyro())); // Reset Gyro
                m_driverController.y().toggleOnTrue(new IntakeCommand(m_robotIntake, -1500)); // Intake
                m_driverController.a().whileTrue(new IntakeCommand(m_robotIntake, 750)); // Outake
                m_driverController.povLeft().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative)); // Toggle
                                                                                                               // Field
                                                                                                               // Relativity
                m_driverController.rightBumper().toggleOnTrue(
                                new ShootWithLimelightCommand(m_robotShooter, m_robotIndexer, 10,
                                                "limelight-main", m_driverController)); // Toggle Limelight Shooter
                                                                                        // Sequence

                m_driverController.leftBumper().toggleOnFalse(new SpindexerCommand(m_robotIndexer));

                // ====================
                // Turret Controls (Josh)
                // ====================
                m_turretController.leftTrigger().whileTrue(new RotateCommand(m_robotTurret, m_turretController)); // Rotate
                // turret
                // counter-clockwise
                m_turretController.rightTrigger().whileTrue(new RotateCommand(m_robotTurret, m_turretController)); // Rotate
                // turret
                // clockwise
                m_turretController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)); // Toggle
                // x-formation
                m_turretController.x().toggleOnTrue(
                                new CalculateTurretPosition(m_robotTurret, "limelight-main", m_alliance.getSelected())); // Start
                // Turret
                // Position
                // Calculation
                m_turretController.leftBumper().toggleOnTrue(
                                new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 3050, m_driverController)); // Toggle
                                                                                                                       // Manual
                // Shooter Sequence
        }

        @Override
        public void periodic() {
                // isLocked = LimelightHelpers.getFiducialID("limelight-main") == 26
                // || LimelightHelpers.getFiducialID("limelight-main") == 21 ||
                // LimelightHelpers.getFiducialID("limelight-main") == 18
                // || LimelightHelpers.getFiducialID("limelight-main") == 5
                // || LimelightHelpers.getFiducialID("limelight-main") == 10
                // || LimelightHelpers.getFiducialID("limelight-main") == 2;

                // turretLocked = isLocked ? "Turret Locked" : "Turret Not Locked";
        }

        // ====================
        // Shuffleboard Control Layout For Drivers (Remove if causes issues)
        // ===================
        private void setupShuffleboard() {
                ShuffleboardTab layoutTab = Shuffleboard.getTab("Control Bindings");

                boolean isLocked = LimelightHelpers.getFiducialID("limelight-main") == 26
                                || LimelightHelpers.getFiducialID("limelight-main") == 21 ||
                                LimelightHelpers.getFiducialID("limelight-main") == 18
                                || LimelightHelpers.getFiducialID("limelight-main") == 5
                                || LimelightHelpers.getFiducialID("limelight-main") == 10
                                || LimelightHelpers.getFiducialID("limelight-main") == 2;

                String turretLocked = isLocked ? "Turret Locked" : "Turret Not Locked";

                Command eightAuto = new PathPlannerAuto("8 Shoot");
                Command sideAuto = new ParallelCommandGroup(
                                new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950, m_driverController),
                                new AutonomousIntakeCommand(m_robotIntake, -1500));
                // new ParallelCommandGroup(
                // new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950,
                // m_driverController),
                // new CalculateTurretPosition(m_robotTurret, "limelight-main",
                // m_alliance.getSelected()));

                m_chooser.setDefaultOption("8 Shoot", eightAuto);
                m_chooser.addOption("Side Shoot", sideAuto);
                m_chooser.addOption("Do Nothing", new WaitCommand(20));

                m_alliance.setDefaultOption("Red", "Red");
                m_alliance.addOption("Blue", "Blue");

                var driverLayout = layoutTab.getLayout("Driver (Stone)", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0);

                driverLayout.addString("B Button", () -> "Reset Gyro");
                driverLayout.addString("Y Button", () -> "Toggle Intake (-1500)");
                driverLayout.addString("A Button", () -> "Hold Outtake (750)");
                driverLayout.addString("POV Left", () -> "Toggle Field Relative");
                driverLayout.addString("Right Bumper", () -> "Limelight Shoot Seq");

                var turretLayout = layoutTab.getLayout("Turret (Josh)", BuiltInLayouts.kList)
                                .withSize(2, 3)
                                .withPosition(2, 0);

                turretLayout.addString("Triggers", () -> "Rotate Turret");
                turretLayout.addString("Right Bumper", () -> "Set X-Pattern");
                turretLayout.addString("X Button", () -> "Auto-Aim (Limelight)");
                turretLayout.addString("Left Bumper", () -> "Manual Shoot Seq");

                layoutTab.add("Alliance", m_alliance)
                                .withPosition(4, 1);
                layoutTab.add("Autonomous", m_chooser)
                                .withPosition(4, 2);

                layoutTab.addString(turretLocked, () -> turretLocked);

                layoutTab.addBoolean("Field Relative", () -> fieldRelative)
                                .withPosition(4, 0)
                                .withWidget("Boolean Box")
                                .withProperties(Map.of("Color when true", "Green", "Color when false", "Red"));
                layoutTab.addDouble("Gyro Degree", () -> m_robotDrive.getHeading())
                                .withPosition(2, 3)
                                .withWidget("Gyro");
                
                layoutTab.addDouble("Indexer Speed", () -> m_robotIndexer.getRPM())
                        .withPosition(4, 4);
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}