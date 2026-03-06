package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climclamSpark;

    public ClimbSubsystem(int canID) {
        climclamSpark = new SparkMax(canID, MotorType.kBrushless);
    }

    public void SetSpeed(double speed) {
        System.out.println("Running climb at " + speed);
        climclamSpark.set(speed);
    }

    public void StopMotor() {
        climclamSpark.set(0);
    }
}
