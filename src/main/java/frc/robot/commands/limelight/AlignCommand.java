package frc.robot.commands.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants.AlignConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignCommand extends Command {
    private final int aprilTagId;

    private final XboxController controller;

    private final String limelightName;

    private final double desiredTargetY;

    private final DriveSubsystem robotDrive;

    /**
     * 
     * @param aprilTagId     The Id for the desired april tag target
     * @param controller     The xbox controller
     * @param limelightName  The name of the limelight assigned on the limelight
     *                       webpage
     * @param desiredTargetY The desired target
     * @param robotDrive     The robot drive subsystem
     */
    public AlignCommand(int aprilTagId, XboxController controller, String limelightName, double desiredTargetY,
            DriveSubsystem robotDrive) {
        this.aprilTagId = aprilTagId;
        this.controller = controller;
        this.limelightName = limelightName;
        this.desiredTargetY = desiredTargetY;
        this.robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    /**
     * 
     * @param aprilTagId     The Id for the desired april tag target
     * @param controller     The xbox controller
     * @param limelightName  The name of the limelight assigned on the limelight
     *                       webpage
     * @param desiredTargetY The desired target
     * @param robotDrive     The robot drive subsystem
     */
    public AlignCommand(int aprilTagId, CommandXboxController controller, String limelightName, double desiredTargetY,
            DriveSubsystem robotDrive) {
        this(aprilTagId, controller.getHID(), limelightName, desiredTargetY, robotDrive);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            robotDrive.drive(0, 0, 0, false, 0);
            System.err.println("No target found!");
            return;
        }

        double tx = LimelightHelpers.getTX(limelightName);
        double ty = LimelightHelpers.getTY(limelightName);

        double kP_Rot = AlignConstants.kP_Rot;
        double kP_Strafe = AlignConstants.kP_Strafe;
        double kP_Range = AlignConstants.kP_Range;

        double rotationOutput = tx * kP_Rot;
        double strafeOutput = tx * kP_Strafe;

        double rangeOutput = (desiredTargetY - ty) * kP_Range;

        if (Math.abs(tx) < 0.5) {
            rangeOutput = 0;
        }

        // Change this for practice based on how fast you want to test.
        rotationOutput = MathUtil.clamp(rotationOutput, -0.1, 0.1);
        strafeOutput = MathUtil.clamp(strafeOutput, -0.1, 0.1);
        rangeOutput = MathUtil.clamp(rangeOutput, -0.1, 0.1);

        if (hasTarget && Math.abs(tx) < 3.0) {
            controller.setRumble(RumbleType.kBothRumble, 0.1);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
        robotDrive.drive(
                rangeOutput * DriveConstants.kMaxSpeedMetersPerSecond, // NEW: Drive
                // Forward/Back
                // 0,
                // 0,
                -strafeOutput * DriveConstants.kMaxSpeedMetersPerSecond,
                -rotationOutput * DriveConstants.kMaxAngularSpeed,
                false,
                0);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
        LimelightHelpers.setPriorityTagID(limelightName, -1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
