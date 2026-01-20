package frc.robot.commands.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleAlignCommand extends Command {
    private final int aprilTagId;
    private final XboxController controller;
    private final String limelightName;
    private final DriveSubsystem robotDrive;

    // --- TUNING SECTION ---
    // If robot spins the WRONG way, change this to TRUE
    private static final boolean INVERT_ROTATION = false; 
    
    // If robot drives AWAY from target, change this to TRUE
    private static final boolean INVERT_FORWARD = true; 

    // Target "ty" (Vertical Offset). 
    // Measure this! Place robot at goal, read 'ty' from dashboard.
    // Example: 0.0 means "Drive until target is centered vertically"
    private double targetTy = 0.0; 

    public SimpleAlignCommand(int aprilTagId, XboxController controller, String limelightName, DriveSubsystem robotDrive) {
        this.aprilTagId = aprilTagId;
        this.controller = controller;
        this.limelightName = limelightName;
        this.robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0); 
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
        
        System.out.println("Alignment Started for Tag " + aprilTagId);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        double tx = LimelightHelpers.getTX(limelightName); // Left/Right error
        double ty = LimelightHelpers.getTY(limelightName); // Distance error

        // POST TO DASHBOARD (Essential for debugging!)
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
        SmartDashboard.putNumber("Limelight/tx", tx);
        SmartDashboard.putNumber("Limelight/ty", ty);

        if (!hasTarget) {
            // If no target, stop safely.
            robotDrive.drive(0, 0, 0, false, 0);
            return;
        }

        // --- 1. ROTATION CALCULATION (Turn to face target) ---
        double kP_Rot = 0.0015; // Strength of turning
        double rotSpeed = tx * kP_Rot;
        
        if (INVERT_ROTATION) rotSpeed = -rotSpeed;

        // --- 2. FORWARD CALCULATION (Drive to correct distance) ---
        double kP_Dist = 0.01; // Strength of driving
        double distError = targetTy - ty; // Difference between desired angle and current angle
        double forwardSpeed = distError * kP_Dist;

        if (INVERT_FORWARD) forwardSpeed = -forwardSpeed;

        // --- 3. CLAMP SPEEDS (Safety) ---
        // Keeps the robot from going too fast while testing
        rotSpeed = MathUtil.clamp(rotSpeed, -0.3, 0.3);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.3, 0.3);

        SmartDashboard.putNumber("Limelight/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("Limelight/RotSpeed", rotSpeed);

        // --- 4. DRIVE ---
        // Note: No "Strafe" (ySpeed) in this simple version to isolate the issue.
        robotDrive.drive(
            -forwardSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            // 0,
            0, 0,
            // -rotSpeed * DriveConstants.kMaxAngularSpeed, 
            false, // Robot Centric usually easiest for alignment
            0
        );
        
        // Rumble when locked on
        if (Math.abs(tx) < 2.0 && Math.abs(distError) < 2.0) {
            controller.setRumble(RumbleType.kBothRumble, 0.4);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        robotDrive.drive(0, 0, 0, false, 0);
        controller.setRumble(RumbleType.kBothRumble, 0);
        LimelightHelpers.setPriorityTagID(limelightName, -1);
        System.out.println("Alignment Ended");
    }
}