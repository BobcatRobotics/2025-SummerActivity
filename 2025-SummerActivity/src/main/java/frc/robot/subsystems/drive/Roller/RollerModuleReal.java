package frc.robot.subsystems.drive.Roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.Constants;
// import frc.robot.subsystems.drive.Roller.RollerModuleIO.RollerModuleIOInputs;
import org.littletonrobotics.junction.Logger;

public class RollerModuleReal implements RollerModuleIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;

  // Control mode objects reused for performance
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public RollerModuleReal(int id, String bus) {
    this.motor = new TalonFX(id, bus);

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 25;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;

    // Apply initial configuration with retry logic
    motor.getConfigurator().apply(config, 0.25);

    // Get references to sensor signals
    relativePosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RollerModuleIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

    if (inputs.appliedVolts > 0) {
      inputs.state = RollerState.FORWARD;
    } else if (inputs.appliedVolts < 0) {
      inputs.state = RollerState.REVERSE;
    } else if (inputs.appliedVolts == 0) {
      inputs.state = RollerState.IDLE;
    } else {
      inputs.state = RollerState.UNKNOWN;
    }
  }

  public void runRoller(double speed) {
    double velocityRotPerSec = Units.radiansToRotations(speed);
    motor.setControl(velocityControl.withVelocity(velocityRotPerSec));
    Logger.recordOutput("/Roller/velocityRotPerSec", velocityRotPerSec);
  }

  public void stopRoller() {
    motor.stopMotor();

    Logger.recordOutput("/Roller/velocityRotPerSec", 0);
  }
}
