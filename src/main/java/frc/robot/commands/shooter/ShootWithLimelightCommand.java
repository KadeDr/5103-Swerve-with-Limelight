package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootWithLimelightCommand extends Command {
    private final int aprilTagId;
    private final String limelightName;
    private final ShootSubsystem shootSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final CommandXboxController controller;

    private boolean hasStarted = false;
    private boolean isReversed = false;

    public ShootWithLimelightCommand(ShootSubsystem shootSubsystem, IndexerSubsystem indexerSubsystem,
            int aprilTagId, String limelightName, CommandXboxController controller) {
        this.shootSubsystem = shootSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.aprilTagId = aprilTagId;
        this.limelightName = limelightName;
        this.controller = controller;

        addRequirements(shootSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initialize shooter!");
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        controller.povLeft().whileTrue(new InstantCommand(() -> isReversed = true));
        controller.povLeft().whileFalse(new InstantCommand(() -> isReversed = false));
        System.out.println("Running shooter!");
        boolean tv = LimelightHelpers.getTV(limelightName);
        double id = LimelightHelpers.getFiducialID(limelightName);
        boolean isCorrectTag = (id == 12 || id == 16);

        if (!tv || !isCorrectTag)
            return;

        double h1 = LimelightConstants.PhysicalConstants.kLimelightHeightInches;
        double h2 = LimelightConstants.PhysicalConstants.kHubAprilTagHeightInches;

        double a1 = LimelightConstants.PhysicalConstants.kLimelightAngleDegrees;
        double tY = LimelightHelpers.getTY(limelightName);

        double distanceInches = ((h2 - h1) / Math.tan(Math.toRadians(a1 - tY))); // Previously 19
        @SuppressWarnings("unused")
        double distanceFeet = (distanceInches / 12);
        double speedRpm = -0.00883 * (distanceInches * distanceInches) + 12.323 * distanceInches + 2003.82;

        shootSubsystem.runVelocity(speedRpm);

        // Remove these lines if the stop while locating breaks
        if (indexerSubsystem.isLocatingTarget) {
            indexerSubsystem.stopMotor();
            return;
        }

        if (!hasStarted) {
            Timer.delay(0.5);
            hasStarted = true;
        }

        if (isReversed) {
            indexerSubsystem.spin(-1500);
        } else {
            indexerSubsystem.spin(1500);
        }
    }

    @Override
    public void end(boolean isFinished) {
        indexerSubsystem.stopMotor();
        shootSubsystem.stopMotor();
    }
}