package frc.robot.commands.shooter;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.commands.indexer.SpinIndexerCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithLimelightCommand extends Command {
    private final int aprilTagId;
    private final String limelightName;
    private final ShooterSubsystem shooterSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    private boolean hasStarted = false;

    public ShootWithLimelightCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
            int aprilTagId, String limelightName) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.aprilTagId = aprilTagId;
        this.limelightName = limelightName;

        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initialize shooter!");
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        System.out.println("Running shooter!");
        boolean tv = LimelightHelpers.getTV(limelightName);
        double id = LimelightHelpers.getFiducialID(limelightName);
        boolean isCorrectTag = (id == 12 || id == 16);

        if (!tv || !isCorrectTag)
            return;

        SmartDashboard.putBoolean("Has proper target", LimelightHelpers.getFiducialID(limelightName) == 12
                || LimelightHelpers.getFiducialID(limelightName) == 16);

        double h1 = LimelightConstants.PhysicalConstants.kLimelightHeightInches;
        double h2 = LimelightConstants.PhysicalConstants.kHubAprilTagHeightInches;

        double a1 = LimelightConstants.PhysicalConstants.kLimelightAngleDegrees;
        double tY = LimelightHelpers.getTY(limelightName);
        
        double distanceInches = ((h2 - h1) / Math.tan(Math.toRadians(a1 - tY))); // Previously 19
        double distanceFeet = (distanceInches / 12);
        double speedRpm = -0.00883 * (distanceInches * distanceInches) + 12.323 * distanceInches + 2003.82;

        SmartDashboard.putNumber("Limelight TY", tY);
        SmartDashboard.putNumber("Target Shooter Speed RPM", speedRpm);
        SmartDashboard.putBoolean("Has started", hasStarted);
        SmartDashboard.putNumber("Robot distance", distanceInches);

        shooterSubsystem.runVelocity(speedRpm);

        if (!hasStarted) {
            Timer.delay(0.5);
            hasStarted = true;
        }

        indexerSubsystem.spin(1500);
    }

    @Override
    public void end(boolean isFinished) {
        indexerSubsystem.stopMotor();
        shooterSubsystem.stopMotor();
    }
}