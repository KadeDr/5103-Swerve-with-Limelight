package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{
    private final ClimbSubsystem m_climb;
    private final double m_speed;

    public ClimbCommand(ClimbSubsystem subsystem, double speed) {
        m_climb = subsystem;
        m_speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_climb.SetSpeed(m_speed);
        System.out.println("Running climb!");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stopping climb!");
        m_climb.StopMotor();
    }
}
