package frc.robot.subsystems.drive.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX armMotor = new TalonFX(9, "rio"); // assuming id is 1
  // Setpoint positions (in rotations)
  private final double kExtendedPosition = 500.0;
  private final double kStowedPosition = 0.0;

  public ArmSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = 75;

    // limits
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    armMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {}

  public void extendArm() {
    armMotor.set(0.177);
  }

  public void stowArm() {
    armMotor.set(-0.19);
  }

  public void stopMotor() {
    armMotor.stopMotor();
  }
}
