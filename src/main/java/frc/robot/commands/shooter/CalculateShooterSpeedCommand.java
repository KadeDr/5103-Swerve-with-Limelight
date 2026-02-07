package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class CalculateShooterSpeedCommand extends Command {
    private double distance;
    private double velocity;
    private double rpm;
    private final String limelightName;

    public CalculateShooterSpeedCommand(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, 9);
    }

    @Override
    public void execute() {
        boolean tv = LimelightHelpers.getTV(limelightName);

        if (!tv && (LimelightHelpers.getFiducialID(limelightName) != 9
                && LimelightHelpers.getFiducialID(limelightName) != 10
                && LimelightHelpers.getFiducialID(limelightName) != 25
                && LimelightHelpers.getFiducialID(limelightName) != 26)) {
            return;
        }

        distance = LimelightHelpers.getTY(limelightName);
        velocity = (1.12 - Math.sqrt(0.5472-0.0208 * distance)) / 0.0104;
        System.out.println(velocity);
        rpm = velocity * 67;

        returnRPM();
    }

    public double returnRPM() {
        return rpm;
    }
}
