// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs.ShooterConfigs;

// public class ShooterSubsystem extends SubsystemBase {
//     private SparkMax m_shooter;
//     private final SparkMax m_turntable;
//     private SparkClosedLoopController m_shooterCLC;
//     private final SparkClosedLoopController m_turntableCLC;
//     private RelativeEncoder m_shooterEncoder;
//     private RelativeEncoder m_turnTableEncoder;

//     public ShooterSubsystem(int shooterCanId, int turnTableCanId) {
//         m_shooter = new SparkMax(shooterCanId, MotorType.kBrushless);
//         m_turntable = new SparkMax(turnTableCanId, MotorType.kBrushless);
//         m_turntableCLC = m_turntable.getClosedLoopController();
//         m_shooterCLC = m_shooter.getClosedLoopController();
//         m_shooterEncoder = m_shooter.getEncoder();
//         m_turnTableEncoder = m_turntable.getEncoder();
//         m_turntable.configure(ShooterConfigs.turntableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         m_shooter.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Turret Position", getPosition());
//         SmartDashboard.putNumber("Shooter Speed", getRPM());
//     }

//     public void runVelocity(double targetRPM) {
//         m_shooterCLC.setSetpoint(targetRPM, ControlType.kVelocity);
//     }

//     public void stopMotor() {
//         m_shooter.set(0);
//     }

//     public void turnTable(double targetPosition) {
//         // if (targetPosition < ShooterConstants.kMaxRotationLeft && targetPosition > ShooterConstants.kMaxRotationRight) { return; }
//         m_turntableCLC.setSetpoint(targetPosition, ControlType.kPosition);
//     }

//     public void turnTableVelocity(double targetSpeed) {
//         m_turntableCLC.setSetpoint(targetSpeed, ControlType.kVelocity);
//     }

//     public double getPosition() {
//         return m_turnTableEncoder.getPosition();
//     }

//     public double getRPM() {
//         return m_shooterEncoder.getVelocity(); 
//     }

//     public void ResetEncoder() {
//         m_turnTableEncoder.setPosition(0);
//     }
// }
