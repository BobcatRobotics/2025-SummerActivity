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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RollerConstants;
import frc.robot.util.TunnableNumber;
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
  private double maxSimVelocity = 511.0; // rotations/sec
  private double maxAcceleration = 100.0; // RPS * RPS
  private final double simLoopPeriodSec = 0.02; // 20ms typical loop time

  TunnableNumber kP;
  TunnableNumber kD;
  TalonFXConfiguration config;
  private double simVoltage = 0.0;

  private FlywheelSim mRollerSim;

  public RollerIOSim() {
    var plant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.001, 25.0);
    mRollerSim = new FlywheelSim(plant, DCMotor.getKrakenX60(1), 0.0);

    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    config.Feedback.SensorToMechanismRatio = 25; // 1:25 gear ratio

    // Velocity PIDs
    kP = new TunnableNumber("/Roller/kP", 0.11);
    kD = new TunnableNumber("/Roller/kD", 0);
    config.Slot0.kP = kP.get();
    config.Slot0.kI = 0;
    config.Slot0.kD = kD.get();
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
    mRollerSim.update(simLoopPeriodSec);
    kP.periodic();
    kD.periodic();
    if (kP.updateConfig) {
      config.Slot0.kP = kP.get();
      rollerMotor.getConfigurator().apply(config);
    }
    if(kD.updateConfig) {
      config.Slot0.kD = kD.get();
      rollerMotor.getConfigurator().apply(config);
    }

    var motorStatus =
        BaseStatusSignal.refreshAll(
            relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.rollerConnected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.rollerPosition = relativePosition.getValueAsDouble();
    var vel = mRollerSim.getAngularVelocity();
    inputs.rollerVelocity = vel.baseUnitMagnitude();
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    // Simulated inputs would be updated here, but for now we leave it empty

    Logger.recordOutput("/Roller/Velocity", motorVelocity.getValueAsDouble());
    Logger.recordOutput(
        "/Roller/TargetVelocityIn", Units.radiansToRotations(RollerConstants.ROLLER_SPEED_IN));
    Logger.recordOutput(
        "/Roller/TargetVelocityOut", Units.radiansToRotations(RollerConstants.ROLLER_SPEED_OUT));
  }

  @Override
  public void runRoller(double speed) {
    mRollerSim.setInputVoltage(speed / maxSimVelocity * 12);
    mRollerSim.update(simLoopPeriodSec);
    var vel = mRollerSim.getAngularVelocity();
    rollerMotor.setControl(mVelocityVoltage.withVelocity(vel));
  }

  @Override
  public void stopRoller() {
    rollerMotor.stopMotor();
    runRoller(0);
    // Simulated stop logic would go here, but for now we leave it empty
  }
}
