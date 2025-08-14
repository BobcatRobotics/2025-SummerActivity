package frc.robot.subsystems.algaeRemover;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
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
import frc.robot.subsystems.arm.ArmModuleIO.ArmModuleIOInputs;

public class AlgaeRemoverModuleSim implements AlgaeRemoverIO {

  private final TalonFX armMotor;
  private final TalonFX rollerMotor;
  private final StatusSignal<Angle> armRelativePosition;
  private final StatusSignal<AngularVelocity> armMotorVelocity;
  private final StatusSignal<Voltage> armMotorAppliedVolts;
  private final StatusSignal<Current> armMotorCurrent;

  private final StatusSignal<Angle> rollerRelativePosition;
  private final StatusSignal<AngularVelocity> rollerMotorVelocity;
  private final StatusSignal<Voltage> rollerMotorAppliedVolts;
  private final StatusSignal<Current> rollerMotorCurrent;

  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration armConfig;
  private final TalonFXConfiguration rollerConfig;

  // Control mode objects reused for performance
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public AlgaeRemoverModuleSim(int armId,int rollerId, String bus) {
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


    // Apply initial configuration with retry logic
    armMotor.getConfigurator().apply(armConfig, 0.25);

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

    rollerConfig.Slot0.kS = 0.1;
    rollerConfig.Slot0.kV = 0.12;
    rollerConfig.Slot0.kP = 0.11;
    

    // Apply initial configuration with retry logic
    rollerMotor.getConfigurator().apply(rollerConfig, 0.25);


    // Get references to sensor signals
    armRelativePosition = armMotor.getPosition();
    armMotorVelocity = armMotor.getVelocity();
    armMotorAppliedVolts = armMotor.getMotorVoltage();
    armMotorCurrent = armMotor.getStatorCurrent();

    // Get references to sensor signals
    rollerRelativePosition = rollerMotor.getPosition();
    rollerMotorVelocity = rollerMotor.getVelocity();
    rollerMotorAppliedVolts = rollerMotor.getMotorVoltage();
    rollerMotorCurrent = rollerMotor.getStatorCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armRelativePosition, armMotorVelocity, armMotorAppliedVolts, armMotorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(armMotor);

    
    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerRelativePosition, rollerMotorVelocity, rollerMotorAppliedVolts, rollerMotorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  public void updateInputs(AlgaeRemoverIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
          armRelativePosition, armMotorVelocity, armMotorAppliedVolts, armMotorCurrent,rollerRelativePosition, rollerMotorVelocity, rollerMotorAppliedVolts, rollerMotorCurrent);

    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.armPositionRad = Units.rotationsToRadians(armRelativePosition.getValueAsDouble());
    inputs.armVelocityRadPerSec = Units.rotationsToRadians(armMotorVelocity.getValueAsDouble());
    inputs.armAppliedVolts = armMotorAppliedVolts.getValueAsDouble();
    inputs.armCurrentAmps = armMotorCurrent.getValueAsDouble();
    inputs.rollerPositionRad = Units.rotationsToRadians(rollerRelativePosition.getValueAsDouble());
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerMotorVelocity.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerMotorAppliedVolts.getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotorCurrent.getValueAsDouble();

    if (inputs.armAppliedVolts > 0) {
      inputs.armState = AlgaeRemoverArmState.FORWARD;
    } else if (inputs.armAppliedVolts < 0) {
      inputs.armState = AlgaeRemoverArmState.REVERSE;
    } else if (inputs.armAppliedVolts == 0) {
      inputs.armState = AlgaeRemoverArmState.IDLE;
    } else {
      inputs.armState = AlgaeRemoverArmState.UNKNOWN;
    }

    if (inputs.rollerAppliedVolts > 0) {
      inputs.rollerState = AlgaeRemoverRollerState.FORWARD;
    } else if (inputs.rollerAppliedVolts < 0) {
      inputs.rollerState = AlgaeRemoverRollerState.REVERSE;
    } else if (inputs.rollerAppliedVolts == 0) {
      inputs.rollerState = AlgaeRemoverRollerState.IDLE;
    } else {
      inputs.rollerState = AlgaeRemoverRollerState.UNKNOWN;
    }
  }

  public void runArm(double positionInRotations) {
    // var request = positionControl.withPosition(positionInRotations);
    // motor.setControl(request);
    armMotor.set(positionInRotations);
    Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
  }

  public void stopArm() {
    armMotor.stopMotor();
    Logger.recordOutput("/Arm/positionInRotations", 0);
  }
  public void stopRoller() {
    rollerMotor.stopMotor();
    Logger.recordOutput("/Arm/positionInRotations", 0);
  }
}
