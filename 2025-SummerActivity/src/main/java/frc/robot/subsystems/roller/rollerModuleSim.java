package frc.robot.subsystems.roller;

import javax.sound.sampled.Line;

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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.TunableDouble;

public class RollerModuleSim implements RollerModuleIO {

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

  // Use onboard Talon PID in real; use software PID in sim for clarity
  private final PIDController simPid = new PIDController(0.0, 0.0, 0.00);
  // Flywheel/roller model
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
  private static final double GEAR_RATIO = 25.0; // adjust if using reduction
  private static final double MOI = 0.0009; // kg·m² — estimate, refine later
  // Simulate a flywheel made from 4 Colson Wheels being directly driven by a Neo
  // motor
  private final FlywheelSim m_flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          MOTOR, MOI, GEAR_RATIO),
      MOTOR);
  private final TunableDouble kP = new TunableDouble("/Roller/Config/kP", 0.1);
  private final TunableDouble kI = new TunableDouble("/Roller/Config/kI", 0.0);
  private final TunableDouble kD = new TunableDouble("/Roller/Config/kD", 0.0);
  private final TunableDouble kV = new TunableDouble("/Roller/Config/kV", 0.0);
  private TalonFXSimState motorSim;

  private double simulatedPosition = 0.0;
  private double simulatedVelocity = 0.0;
  private double goalVelocity = 0.0;
  private double maxSimVelocity = 10.0; // rotations/sec
  private double maxAcceleration = 100.0; // RPS * RPS
  private final double simLoopPeriodSec = 0.02; // 20ms typical loop time

  public RollerModuleSim(int id, String bus) {

    this.motor = new TalonFX(id, bus);

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 25;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kP.get();
    config.Slot0.kD = kP.get();
    config.Slot0.kV = kP.get();
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

  public void updateTunnables() {
    if (kP.check()) {
      config.Slot0.kP = kP.get();
    }
    if (kI.check()) {
      config.Slot0.kI = kI.get();
    }
    if (kP.check()) {
      config.Slot0.kD = kD.get();
    }
    if (kP.check()) {
      config.Slot0.kV = kV.get();
    }
    motor.getConfigurator().apply(config, 0.25);
  }

  @Override
  public void updateInputs(RollerModuleIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(
        relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);
    motorSim = motor.getSimState();
    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = relativePosition.getValueAsDouble();
    inputs.velocityRadPerSec = motorVelocity.getValueAsDouble();
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

    // AdvantageKit logging
    Logger.recordOutput("/Roller/GoalRotPerSec", goalVelocity);
    Logger.recordOutput("/Roller/MeasuredRotPerSec", relativePosition.getValueAsDouble());
    Logger.recordOutput("/Roller/ErrorRotPerSec", goalVelocity - relativePosition.getValueAsDouble());
  }

  public void periodic(){
    updateTunnables();
    motorSim = motor.getSimState();
    // Use a software PID + feedforward to produce a voltage request in sim (mirrors
    // real firmware behavior)
    double measured = motor.getVelocity().getValueAsDouble();
    double ffVolts = 12.0 * (0.12 * goalVelocity); // simple kV-style FF example
    double pidVolts = simPid.calculate(measured, goalVelocity);
    double volts = Math.max(-12.0, Math.min(12.0, ffVolts + pidVolts));
    m_flywheelSim.setInputVoltage(volts);
    m_flywheelSim.update(simLoopPeriodSec);

    // Push sim signals back into Phoenix 6 sim side so telemetry & getVelocity() look real
    double shaftRPS = m_flywheelSim.getAngularVelocityRPM() / 60.0;
    simulatedVelocity = shaftRPS * GEAR_RATIO;
    simulatedPosition += simulatedVelocity * simLoopPeriodSec;
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setRotorVelocity(simulatedVelocity); 
    motorSim.setRawRotorPosition(simulatedPosition);
    // Log sim internals to AdvantageKit (super helpful for tuning)
    Logger.recordOutput("/Roller/VoltageCmd", volts);
    Logger.recordOutput("/Roller/FFVolts", ffVolts);
    Logger.recordOutput("/Roller/PIDVolts", pidVolts);
    Logger.recordOutput("/RollerSim/ShaftRPS", shaftRPS);
  }

  public void runRoller(double speed) {
    goalVelocity = Units.radiansToRotations(speed);
    motor.setControl(velocityControl.withVelocity(goalVelocity));
  }

  public void stopRoller() {
    // runRoller(0);
    motor.stopMotor();
    runRoller(0);
  }
}
