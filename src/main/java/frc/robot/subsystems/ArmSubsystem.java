package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  private final TalonFX armMotor = new TalonFX(10,"rio");
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private final double kExtendedPosition = 5;
  private final double kStowedPosition = 0.0;

  public ArmSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = 25;

    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.8;
    config.Slot0 = Slot0Configs;

    armMotor.getConfigurator().apply(config);
  }


  //Method to stop Motor
  public void stopMotor(){
    armMotor.stopMotor();
  }

  //Method to set arm position
  public void extend(){
    positionRequest.withPosition(kExtendedPosition);
    armMotor.setControl(positionRequest);
  }
  public void stow(){
    positionRequest.withPosition(kStowedPosition);
    armMotor.setControl(positionRequest);
  }

    @Override
  public void periodic() {
    Logger.recordOutput("armPosition",armMotor.getPosition().getValueAsDouble());

    // This method will be called once per scheduler run
  }
}
