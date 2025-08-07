package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.RollerConstants;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX rollerMotor;
  final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  public RollerIOTalonFX(int motorID) {
    rollerMotor = new TalonFX(motorID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;

    // Velocity PIDs
    config.Slot0.kP = 0.11; // TODO tune this
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    rollerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(RollerIOinputs inputs) {
    inputs.rollerConnected = rollerMotor.isConnected();
    inputs.rollerPosition = rollerMotor.getPosition().getValueAsDouble();
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void runRoller(double speed) {
    rollerMotor.setControl(mVelocityVoltage.withVelocity(speed));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
