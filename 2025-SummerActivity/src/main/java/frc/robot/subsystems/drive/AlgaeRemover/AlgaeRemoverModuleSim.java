package frc.robot.subsystems.drive.AlgaeRemover;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import org.littletonrobotics.junction.Logger;

public class AlgaeRemoverModuleSim implements AlgaeRemoverIO {
  private final TalonFX armMotor;
  private final TalonFX rollerMotor;

  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armVoltage;
  private final StatusSignal<Current> armCurrent;
  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerVoltage;
  private final StatusSignal<Current> rollerCurrent;

  private final Debouncer debouncer = new Debouncer(0.5);
  private final TalonFXConfiguration armConfig;
  private final TalonFXConfiguration rollerConfig;

  // private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  // private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public AlgaeRemoverModuleSim(int armId, int rollerId, String bus) {
    this.armMotor = new TalonFX(armId, bus);
    this.rollerMotor = new TalonFX(rollerId, bus);

    armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = Constants.AlgaeRemoverConstants.ARM_STATOR_LIMIT;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = Constants.AlgaeRemoverConstants.ARM_STATOR_LIMIT;
    armConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

    armConfig.Slot0.kP = 0.11;

    armMotor.getConfigurator().apply(armConfig, 0.25);

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

    rollerConfig.Slot0.kS = 0.1;
    rollerConfig.Slot0.kV = 0.12;
    rollerConfig.Slot0.kP = 0.11;

    rollerMotor.getConfigurator().apply(rollerConfig, 0.25);

    armPosition = armMotor.getPosition();
    armVelocity = armMotor.getVelocity();
    armVoltage = armMotor.getMotorVoltage();
    armCurrent = armMotor.getStatorCurrent();

    rollerPosition = rollerMotor.getPosition();
    rollerVelocity = rollerMotor.getVelocity();
    rollerVoltage = rollerMotor.getMotorVoltage();
    rollerCurrent = rollerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armPosition, armVelocity, armVoltage, armCurrent);
    ParentDevice.optimizeBusUtilizationForAll(armMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerPosition, rollerVelocity, rollerVoltage, rollerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  public void updateInputs(AlgaeRemoverIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            armPosition,
            armVelocity,
            armVoltage,
            armCurrent,
            rollerPosition,
            rollerVelocity,
            rollerVoltage,
            rollerCurrent);

    inputs.connected = debouncer.calculate(motorStatus.isOK());
    inputs.armPositionRad = Units.rotationsToRadians(armPosition.getValueAsDouble());
    inputs.armVelocityRadPerSec = Units.rotationsToRadians(armVelocity.getValueAsDouble());
    inputs.armVolts = armVoltage.getValueAsDouble();
    inputs.armAmps = armCurrent.getValueAsDouble();
    inputs.rollerPositionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
    inputs.rollerVolts = rollerVoltage.getValueAsDouble();
    inputs.rollerAmps = rollerCurrent.getValueAsDouble();

    if (inputs.armVolts > 0) {
      inputs.armState = AlgaeRemoverArmState.FORWARD;
    } else if (inputs.armVolts < 0) {
      inputs.armState = AlgaeRemoverArmState.REVERSE;
    } else if (inputs.armVolts == 0) {
      inputs.armState = AlgaeRemoverArmState.IDLE;
    } else {
      inputs.armState = AlgaeRemoverArmState.UNKNOWN;
    }

    if (inputs.rollerVolts > 0) {
      inputs.rollerState = AlgaeRemoverRollerState.FORWARD;
    } else if (inputs.rollerVolts < 0) {
      inputs.rollerState = AlgaeRemoverRollerState.REVERSE;
    } else if (inputs.rollerVolts == 0) {
      inputs.rollerState = AlgaeRemoverRollerState.IDLE;
    } else {
      inputs.rollerState = AlgaeRemoverRollerState.UNKNOWN;
    }
  }

  public void runRoller(double positionInRotations) {
    var request = positionControl.withPosition(positionInRotations);
    rollerMotor.setControl(request);
    rollerMotor.set(positionInRotations);
    Logger.recordOutput("/AlgaeRemover/Roller/positionInRotations", positionInRotations);
  }

  public void runArm(double positionInRotations) {
    var request = positionControl.withPosition(positionInRotations);
    armMotor.setControl(request);
    armMotor.set(positionInRotations);
    Logger.recordOutput("/AlgaeRemover/Arm/positionInRotations", positionInRotations);
  }

  public void stopArm() {
    armMotor.stopMotor();
    Logger.recordOutput("/AlgaeRemover/Arm/positionInRotations", 0);
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
    Logger.recordOutput("/AlgaeRemover/Roller/positionInRotations", 0);
  }
}
