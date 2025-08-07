package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmModuleIO.ArmModuleIOInputs;

public class ArmModuleSim implements ArmModuleIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;

  // Control mode objects reused for performance
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private PositionVoltage positionControl = new PositionVoltage(0);
  private VelocityVoltage velocityControl = new VelocityVoltage(0);

  // Simulation
  private final TalonFXSimState simState;

  private double simulatedPosition = 0.0;
  private double simulatedVelocity = 0.0;
  private double maxSimVelocity = 10.0; // rotations/sec
  private double maxAcceleration = 100.0; // RPS * RPS
  private final double simLoopPeriodSec = 0.02; // 20ms typical loop time

  public ArmModuleSim(int id, String bus) {

    this.motor = new TalonFX(id, bus);

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 75;
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

    simState = motor.getSimState();
  }

  @Override
  public void updateInputs(ArmModuleIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

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
    double delta = Math.signum(error)
            * Math.min(Math.abs(error), maxSimVelocity * simLoopPeriodSec);

    simulatedPosition += delta;

    simState.setRawRotorPosition(simulatedPosition);
    simState.setRotorVelocity(delta / simLoopPeriodSec);
    simState.setSupplyVoltage(12.0); // simulate battery voltage

    
    Logger.recordOutput("/Arm/positionInRotations", simulatedPosition);
  }

  public void stopArm() {
    //runRoller(0);
    motor.stopMotor();
    double error = simulatedPosition;
    double delta = Math.signum(error)
            * Math.min(Math.abs(error), maxSimVelocity * simLoopPeriodSec);

    simulatedPosition += delta;

    simState.setRawRotorPosition(simulatedPosition);
    simState.setRotorVelocity(delta / simLoopPeriodSec);
    simState.setSupplyVoltage(12.0); // simulate battery voltage

    
    Logger.recordOutput("/Arm/velocityRotPerSec", simulatedPosition);
  }
}
