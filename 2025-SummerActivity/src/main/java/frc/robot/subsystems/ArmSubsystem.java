package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX armMotor;
  // private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);

  public ArmSubsystem() {
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.ARM_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.ARM_MOTOR_STATOR_CURRENT_LIMIT;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25; // TODO tune this
    config.Feedback.SensorToMechanismRatio = 75;

    // Velocity PIDs
    config.Slot0.kP = 0.11; // TODO tune this
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    armMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {}

  // public void runArm(double speed) {
  //   armMotor.setControl(m_velocityVoltage.withVelocity(speed));
  // }

  public void positionArm(double pos) {
    double desiredRotations = pos * ArmConstants.ARM_PID_POSITION;
    if (Math.abs(desiredRotations) <= 0.1) {
      desiredRotations = 0;
    }
    armMotor.setControl(mPositionVoltage.withPosition(desiredRotations));
  }

  public void stopArm() {
    armMotor.stopMotor();
  }
}
