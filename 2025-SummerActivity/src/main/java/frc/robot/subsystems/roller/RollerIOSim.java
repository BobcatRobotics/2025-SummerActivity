package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import frc.robot.Constants.RollerConstants;

public class RollerIOSim implements RollerIO {
  private final TalonFX rollerMotor;
  final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private final TalonFXSimState simState;

  public RollerIOSim() {
    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);
    var talonFXConfigurator = rollerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

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

    simState = rollerMotor.getSimState();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerConnected = rollerMotor.isConnected();
    inputs.rollerPosition = rollerMotor.getPosition().getValueAsDouble();
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    // Simulated inputs would be updated here, but for now we leave it empty
  }

  @Override
  public void runRoller(double speed) {
    rollerMotor.setControl(mVelocityVoltage.withVelocity(speed));
    // Simulated roller control logic would go here, but for now we leave it empty
  }

  @Override
  public void stopRoller() {
    rollerMotor.stopMotor();
    // Simulated stop logic would go here, but for now we leave it empty
  }

}
