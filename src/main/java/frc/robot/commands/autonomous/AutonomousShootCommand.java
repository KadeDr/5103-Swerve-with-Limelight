package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class AutonomousShootCommand extends Command {
    private final ShootSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final double m_targetSpeed;

    public AutonomousShootCommand(ShootSubsystem shooter, IndexerSubsystem indexer, double speed) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_targetSpeed = speed;
        
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
        if (m_indexer.isLocatingTarget) {
            m_indexer.stopMotor();
            return;
        }

        m_indexer.spin(1500);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopMotor();
        m_indexer.stopMotor();
    }
}