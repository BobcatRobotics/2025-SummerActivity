package frc.robot.subsystems.drive.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmModuleSim implements ArmModuleIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> amps;
  private final Debouncer debouncer = new Debouncer(0.5);
  private final TalonFXConfiguration config;

  private final TalonFXSimState simState;

  private double simulatedPosition = 0.0;
  private double simulatedVelocity = 0.0;
  private double maxSimVelocity = 10.0;
  private double maxAcceleration = 100.0;
  private final double simLoopPeriodSec = 0.02;

  public ArmModuleSim(int id, String bus) {

    this.motor = new TalonFX(id, bus);

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 75;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;

    motor.getConfigurator().apply(config, 0.25);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage, amps);
    ParentDevice.optimizeBusUtilizationForAll(motor);

    simState = motor.getSimState();
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
    double error = positionInRotations - simulatedPosition;
    double delta =
        Math.signum(error) * Math.min(Math.abs(error), maxSimVelocity * simLoopPeriodSec);

    simulatedPosition += delta;

    simState.setRawRotorPosition(simulatedPosition);
    simState.setRotorVelocity(delta / simLoopPeriodSec);
    simState.setSupplyVoltage(12.0);

    Logger.recordOutput("/Arm/positionInRotations", simulatedPosition);
  }

  public void stopArm() {
    motor.stopMotor();
    double error = simulatedPosition;
    double delta =
        Math.signum(error) * Math.min(Math.abs(error), maxSimVelocity * simLoopPeriodSec);

    simulatedPosition += delta;

    simState.setRawRotorPosition(simulatedPosition);
    simState.setRotorVelocity(delta / simLoopPeriodSec);
    simState.setSupplyVoltage(12.0);

    Logger.recordOutput("/Arm/velocityRotPerSec", simulatedPosition);
  }
}
