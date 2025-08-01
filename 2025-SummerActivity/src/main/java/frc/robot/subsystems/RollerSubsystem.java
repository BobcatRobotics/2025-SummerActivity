package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  private final TalonFX rollerMotor;

  public RollerSubsystem() {

    // Set up the roller motor as a brushed motor
    // String Busname = ""; you dont need this, it defaults to ""
    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    config.Slot0 = slot0Configs;
    rollerMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {}

  /**
   * This is a method that makes the roller move at your desired speed Positive values make it spin
   * forward and negative values spin it in reverse
   *
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runRoller(double speed) {
    // create a velocity closed-loop request, voltage output, slot 0 configs
    rollerMotor.setControl(m_request.withVelocity(speed));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
