package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Configs.ShooterConfigs;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex m_sparkFlex;
    private final SparkMax m_sparkMax;
    private final SparkClosedLoopController m_flexCLC;
    private final SparkClosedLoopController m_sparkCLC;
    private AbsoluteEncoder m_turnTableAbsoluteEncoder;
    private RelativeEncoder m_turnTableEncoder;

    public ShooterSubsystem(int shooterCanId, int turnTableCanId) {
        m_sparkFlex = new SparkFlex(shooterCanId, MotorType.kBrushless);
        m_sparkMax = new SparkMax(turnTableCanId, MotorType.kBrushless);
        m_sparkCLC = m_sparkMax.getClosedLoopController();
        m_flexCLC = m_sparkFlex.getClosedLoopController();
        m_turnTableAbsoluteEncoder = m_sparkMax.getAbsoluteEncoder();
        m_turnTableEncoder = m_sparkMax.getEncoder();
        m_sparkMax.configure(ShooterConfigs.turnTableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_sparkFlex.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position", getPosition());
    }

    public void runVelocity(double targetRPM) {
        m_flexCLC.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    public void stopMotor() {
        m_sparkFlex.set(0);
    }

    public void turnTable(double targetPosition) {
        // if (targetPosition < ShooterConstants.kMaxRotationLeft && targetPosition > ShooterConstants.kMaxRotationRight) { return; }
        m_sparkCLC.setSetpoint(targetPosition, ControlType.kPosition);
    }

    //public void stopTurnTable() {
    //    m_sparkMax.set(0);
    //}

    public double getPosition() {
        return m_turnTableEncoder.getPosition();
    }
}
