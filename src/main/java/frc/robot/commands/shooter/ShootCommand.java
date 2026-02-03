// IntakeCommand.java
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final double m_targetSpeed;

    // REMOVE SparkFlex from the arguments!
    public ShootCommand(ShooterSubsystem subsystem, double speed) {
        m_shooter = subsystem;
        m_targetSpeed = speed;
        
        // This requirement is now guaranteed to work because we only use the subsystem
        addRequirements(subsystem); 
    }

    @Override
    public void initialize() {
        // Just call the method in the subsystem
        m_shooter.runVelocity(m_targetSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopMotor();
    }
}