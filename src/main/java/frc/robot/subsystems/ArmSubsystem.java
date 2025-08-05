package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  private final TalonFX m_armMotor = new TalonFX(2,"rio");

  public ArmSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 30;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_armMotor.getConfigurator().apply(config);
  }
  //Position Voltage request
  private final PositionVoltage m_armRequest = new PositionVoltage(0);

  //Method to stop Motor
  public void stopMotor(){
    m_armMotor.stopMotor();
  }

  //Method to set arm position
  public void setPosition(){
    m_armMotor.setControl(m_armRequest);
  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
