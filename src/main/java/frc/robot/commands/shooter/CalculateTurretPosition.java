package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class CalculateTurretPosition extends Command {
    private final TurretSubsystem m_turretSubsystem;
    private final String m_limelightName;
    private boolean isTurning;

    public CalculateTurretPosition(TurretSubsystem turretSubsystem, String limelightName) {
        m_turretSubsystem = turretSubsystem;
        m_limelightName = limelightName;
        addRequirements(m_turretSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(m_limelightName, 0);
        m_turretSubsystem.reconfigureTurntable(ShooterConfigs.positionTurntableConfig);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Turntable Turning", isTurning);
        boolean tv = LimelightHelpers.getTV(m_limelightName);

        if (!tv) {
            // isTurning = false;
            return;
        }

        int[] priorityTags = { 2, 5, 10, 18, 21, 26 };
        int[] parallelTags = { 11, 8, 9, 27, 24, 25 };
        int currentId = (int) LimelightHelpers.getFiducialID(m_limelightName);

        for (int id = 0; id < parallelTags.length; id++) {
            if (currentId == parallelTags[id] || currentId == priorityTags[id]) {
                LimelightHelpers.setPriorityTagID(m_limelightName, priorityTags[id]);
            }
        }

        isTurning = true;

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