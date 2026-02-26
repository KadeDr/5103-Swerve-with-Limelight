package frc.robot.commands.shooter;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final CommandXboxController m_controller;
    private double m_targetSpeed;

    public RotateCommand(ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
        m_shooterSubsystem = shooterSubsystem;
        m_controller = controller;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        if (m_controller.getLeftTriggerAxis() > m_controller.getRightTriggerAxis()) {
            m_targetSpeed = -m_controller.getLeftTriggerAxis() * 100;
        } else {
            m_targetSpeed = m_controller.getRightTriggerAxis() * 100;
        }

        System.out.println("Running rotate command!" + m_targetSpeed);

        // if (m_shooterSubsystem.getPosition() >= ShooterConstants.kMaxRotationLeft || m_shooterSubsystem.getPosition() <= ShooterConstants.kMaxRotationRight) return;
        m_shooterSubsystem.turnTableVelocity(m_targetSpeed);
    }

    @Override
    public void end(boolean isFinished) {
        // Check if error is small (e.g., within 2 degrees)
        m_shooterSubsystem.turnTableVelocity(0);
    }
}
