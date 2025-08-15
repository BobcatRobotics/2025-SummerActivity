package frc.robot.subsystems.drive.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ArmModuleReal implements ArmModuleIO {
  private final TalonFX motor;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> amps;
  private final Debouncer debouncer = new Debouncer(0.5);
  private final TalonFXConfiguration config;

  // private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  // private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public ArmModuleReal(int id, String bus) {
    this.motor = new TalonFX(9, "rio");

    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config, 0.25);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage, amps);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(ArmModuleIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(position, velocity, voltage, amps);

    inputs.connected = debouncer.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadperSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.Amps = amps.getValueAsDouble();

    if (inputs.appliedVolts > 0) {
      inputs.state = ArmState.FORWARD;
    } else if (inputs.appliedVolts < 0) {
      inputs.state = ArmState.REVERSE;
    } else if (inputs.appliedVolts == 0) {
      inputs.state = ArmState.IDLE;
    } else {
      inputs.state = ArmState.UNKNOWN;
    }
  }

  public void runArm(double positionInRotations) {
    var request = velocityControl.withVelocity(positionInRotations);
    motor.setControl(request);
    motor.set(positionInRotations);
    Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
  }

  public void stopArm() {
    motor.stopMotor();
    Logger.recordOutput("/Arm/positionInRotations", 0);
  }
}
