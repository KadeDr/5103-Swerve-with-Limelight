package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class RotateCommand extends Command {
    private final TurretSubsystem m_TurretSubsystem;
    private final CommandXboxController m_controller;
    private double m_targetSpeed;

    public RotateCommand(TurretSubsystem TurretSubsystem, CommandXboxController controller) {
        m_TurretSubsystem = TurretSubsystem;
        m_controller = controller;
        addRequirements(m_TurretSubsystem);
    }

    @Override
    public void initialize() {
        m_TurretSubsystem.reconfigureTurntable(ShooterConfigs.velocityTurntableConfig);
    }

    @Override
    public void execute() {
        if (m_controller.getLeftTriggerAxis() > m_controller.getRightTriggerAxis()) {
            m_targetSpeed = -m_controller.getLeftTriggerAxis() * 100;
        } else {
            m_targetSpeed = m_controller.getRightTriggerAxis() * 100;
        }

        if (m_TurretSubsystem.getPosition() <= ShooterConstants.kMaxRotationLeft
                || m_TurretSubsystem.getPosition() >= ShooterConstants.kMaxRotationRight) {
            return;
        }

        m_TurretSubsystem.turnTableVelocity(m_targetSpeed);
    }

    @Override
    public void end(boolean isFinished) {
        m_TurretSubsystem.turnTableVelocity(0);
    }
}
