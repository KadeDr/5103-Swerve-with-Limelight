package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class AutonomousShootCommand extends Command {
    private final ShootSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final double m_targetSpeed;
    private final CommandXboxController controller;

    private boolean isReversed = false;

    public AutonomousShootCommand(ShootSubsystem shooter, IndexerSubsystem indexer, double speed, CommandXboxController controller) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_targetSpeed = speed;
        this.controller = controller;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.runVelocity(m_targetSpeed);
        Timer.delay(0.55);
        m_indexer.spin(1500);
    }

    @Override
    public void execute() {
        controller.leftBumper().whileTrue(new InstantCommand(() -> isReversed = true));
        controller.leftBumper().whileFalse(new InstantCommand(() -> isReversed = false)); 
        // if (m_indexer.isLocatingTarget) {
        //     m_indexer.stopMotor();
        //     return;
        // }

        if (isReversed) {
            m_indexer.spin(-1500);
        } else {
            m_indexer.spin(1500);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopMotor();
        m_indexer.stopMotor();
    }
}