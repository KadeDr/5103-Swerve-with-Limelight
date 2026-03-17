package frc.robot.commands.shooter;

import java.util.Properties;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class CalculateTurretPosition extends Command {
    private final TurretSubsystem m_turretSubsystem;
    private final String m_limelightName;
    private final String m_alliance;
    private boolean isTurning;

    public CalculateTurretPosition(TurretSubsystem turretSubsystem, String limelightName, String alliance) {
        m_turretSubsystem = turretSubsystem;
        m_limelightName = limelightName;
        m_alliance = alliance;
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(m_limelightName, 0);
        m_turretSubsystem.reconfigureTurntable(ShooterConfigs.positionTurntableConfig);
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("Turntable Turning", isTurning);
        boolean tv = LimelightHelpers.getTV(m_limelightName);

        if (!tv) {
            isTurning = false;
            return;
        }

        boolean targetAprilTags = LimelightHelpers.getFiducialID(m_limelightName) == 18
                || LimelightHelpers.getFiducialID(m_limelightName) == 21
                || LimelightHelpers.getFiducialID(m_limelightName) == 26
                || LimelightHelpers.getFiducialID(m_limelightName) == 2
                || LimelightHelpers.getFiducialID(m_limelightName) == 5
                || LimelightHelpers.getFiducialID(m_limelightName) == 10;

        int[] bluePriorityTags = { 18, 21, 26 };
        int[] blueParallelTags = { 27, 24, 25 };
        int[] redPriorityTags = { 2, 5, 10 };
        int[] redParallelTags = { 11, 8, 9 };
        int currentId = (int) LimelightHelpers.getFiducialID(m_limelightName);
        double targetAprilTag = 0;

        if (m_alliance == "Blue") {
            for (int id = 0; id < blueParallelTags.length; id++) {
                if (currentId == blueParallelTags[id] || currentId == bluePriorityTags[id]) {
                    LimelightHelpers.setPriorityTagID(m_limelightName, bluePriorityTags[id]);
                    targetAprilTag = bluePriorityTags[id];
                }
            }
        } else if (m_alliance == "Red") {
            for (int id = 0; id < redParallelTags.length; id++) {
                if (currentId == redParallelTags[id] || currentId == redPriorityTags[id]) {
                    LimelightHelpers.setPriorityTagID(m_limelightName, redPriorityTags[id]);
                    targetAprilTag = redPriorityTags[id];
                }
            }
        }

        // boolean properTargetFound = false;

        if (!targetAprilTags) {
            return;
        }

        // if (!properTargetFound) {
        // return;
        // }

        isTurning = true;

        // ShuffleboardTab layoutTab = Shuffleboard.getTab("Control Bindings");
        // layoutTab.addBoolean("Turntable Turning", () -> isTurning);

        double tx = -LimelightHelpers.getTX(m_limelightName);

        double turretPosition = m_turretSubsystem.getPosition();
        double targetPosition = turretPosition + tx;

        double buffer = 2.0;
        double maxLeft = ShooterConstants.kMaxRotationLeft;
        double maxRight = ShooterConstants.kMaxRotationRight;

        if (targetPosition < (maxLeft - buffer)) {
            // Target is too far left, flip all the way to the right
            targetPosition = maxRight;
        } else if (targetPosition > (maxRight + buffer)) {
            // Target is too far right, flip all the way to the left
            targetPosition = maxLeft;
        } else {
            // If we are within the "buffer" zone but haven't crossed the snap point,
            // clamp it so we don't strain the wires/motors.
            targetPosition = Math.max(maxLeft, Math.min(maxRight, targetPosition));
        }

        m_turretSubsystem.turnTable(targetPosition);
    }

    @Override
    public void end(boolean isFinished) {
        m_turretSubsystem.stopMotors();
    }
}