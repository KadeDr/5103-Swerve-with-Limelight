package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
        LimelightHelpers.setPriorityTagID(m_limelightName, 12);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Turntable Turning", isTurning);
        boolean tv = LimelightHelpers.getTV(m_limelightName);

        if (!tv) {
            // isTurning = false;
            return;
        }

        isTurning = true;

        double tx = -LimelightHelpers.getTX(m_limelightName);

        double turretPosition = m_turretSubsystem.getPosition();
        double targetPosition = turretPosition + tx;

        double maxRotationLeft = ShooterConstants.kMaxRotationLeft;
        double maxRotationRight = ShooterConstants.kMaxRotationRight;

        if (targetPosition < maxRotationLeft) {
            targetPosition = maxRotationRight;
        } else if (targetPosition > maxRotationRight) {
            targetPosition = maxRotationLeft;
        }

        m_turretSubsystem.turnTable(targetPosition);
    }

    @Override
    public void end(boolean isFinished) {
        m_turretSubsystem.stopMotors();
    }
}