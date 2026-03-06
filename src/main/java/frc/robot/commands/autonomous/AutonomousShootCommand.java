// IntakeCommand.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class AutonomousShootCommand extends Command {
    private final ShootSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;
    private final double m_targetSpeed;

    // REMOVE SparkFlex from the arguments!
    public AutonomousShootCommand(ShootSubsystem shooter, IndexerSubsystem indexer, double speed) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_targetSpeed = speed;
        
        // This requirement is now guaranteed to work because we only use the subsystem
        addRequirements(shooter); 
    }

    @Override
    public void initialize() {
        // Just call the method in the subsystem
        m_shooter.runVelocity(m_targetSpeed);
        Timer.delay(0.55);
        m_indexer.spin(1500);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopMotor();
        m_indexer.stopMotor();
    }
}