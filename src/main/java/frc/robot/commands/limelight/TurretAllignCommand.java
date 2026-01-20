package frc.robot.commands.limelight;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class TurretAllignCommand extends Command {
    private final int aprilTagId;

    private final SparkMax pewpewspark;

    private final String limelightName;
    
    public TurretAllignCommand(int aprilTagId, SparkMax pewpewspark, String limelightName) {
        this.aprilTagId = aprilTagId;
        this.pewpewspark = pewpewspark;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        
        if (!hasTarget) 
        {
            pewpewspark.set(0);
            return;
        }

        double kp_Rotation = 0.003; // 0.0015 // Static variable. Constant
        double tx = LimelightHelpers.getTX(limelightName);
        double rot_speed = kp_Rotation * tx;

        pewpewspark.set(-rot_speed);
    }

    @Override
    public void end(boolean interrupted) {
        pewpewspark.set(0);
        LimelightHelpers.setPriorityTagID(limelightName, -1);
    }
}
