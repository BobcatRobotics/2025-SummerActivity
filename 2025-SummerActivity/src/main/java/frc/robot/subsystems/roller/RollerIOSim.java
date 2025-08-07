package frc.robot.subsystems.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.Constants.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class RollerIOSim implements RollerIO {
  private final TalonFX rollerMotor;
  final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private final TalonFXSimState simState;

  private double simulatedPosition = 0.0;
  private double simulatedVelocity = 0.0;
  // private double maxSimVelocity = 10.0; // rotations/sec
  private double maxAcceleration = 100.0; // RPS * RPS
  private final double simLoopPeriodSec = 0.02; // 20ms typical loop time

  public RollerIOSim() {
    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    config.Feedback.SensorToMechanismRatio = 25; // 1:25 gear ratio

    // Velocity PIDs
    config.Slot0.kP = 0.11; // TODO tune this
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    rollerMotor.getConfigurator().apply(config);

    // Get references to sensor signals
    relativePosition = rollerMotor.getPosition();
    motorVelocity = rollerMotor.getVelocity();
    motorAppliedVolts = rollerMotor.getMotorVoltage();
    motorCurrent = rollerMotor.getStatorCurrent();

    simState = rollerMotor.getSimState();
  }

  @Override
  public void updateInputs(RollerIOinputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.rollerConnected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.rollerPosition = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.rollerVelocity = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    // Simulated inputs would be updated here, but for now we leave it empty

    Logger.recordOutput("/Roller/Velocity", simulatedVelocity);
    Logger.recordOutput("/Roller/TargetVelocityIn", RollerConstants.ROLLER_SPEED_IN);
    Logger.recordOutput("/Roller/TargetVelocityOut", RollerConstants.ROLLER_SPEED_OUT);
  }

  @Override
  public void runRoller(double speed) {
    double velocityRotPerSec = Units.radiansToRotations(speed);
    // Simulate gradual acceleration to the target velocity
    double velocityError = velocityRotPerSec - simulatedVelocity;
    double accel =
        Math.signum(velocityError)
            * Math.min(Math.abs(velocityError), maxAcceleration * simLoopPeriodSec);
    simulatedVelocity += accel;

    // Update position based on simulated velocity
    simulatedPosition += simulatedVelocity * simLoopPeriodSec;

    // Feed simulated values to the motor's sim state
    simState.setRotorVelocity(simulatedVelocity);
    simState.setRawRotorPosition(simulatedPosition);
    simState.setSupplyVoltage(12.0); // simulate battery voltage
    // Simulated roller control logic would go here, but for now we leave it empty
  }

  @Override
  public void stopRoller() {
    rollerMotor.stopMotor();
    // Simulated stop logic would go here, but for now we leave it empty
  }
}
