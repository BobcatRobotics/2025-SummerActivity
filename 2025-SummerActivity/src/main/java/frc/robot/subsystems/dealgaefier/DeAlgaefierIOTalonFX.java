package frc.robot.subsystems.dealgaefier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.DealgaefierConstants;
import org.littletonrobotics.junction.Logger;

public class DeAlgaefierIOTalonFX implements DeAlgaefierIO {

  private final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);
  private final TalonFX armMotor;
  private final TalonFX rollerMotor;

  public DeAlgaefierIOTalonFX() {
    armMotor = new TalonFX(DealgaefierConstants.ARM_MOTOR_ID);

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kG = 0.1; // TODO tune this
    armConfig.Slot0.kP = 0.11;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = DealgaefierConstants.ARM_MOTOR_CURRENT_LIMIT;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.CurrentLimits.StatorCurrentLimit =
        DealgaefierConstants.ARM_MOTOR_STATOR_CURRENT_LIMIT;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DealgaefierConstants.ARM_MIN_POSITION;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DealgaefierConstants.ARM_MAX_POSITION;
    armMotor.getConfigurator().apply(armConfig);

    rollerMotor = new TalonFX(DealgaefierConstants.ROLLER_MOTOR_ID);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = 0.11; // TODO tune this
    rollerConfig.Slot0.kI = 0;
    rollerConfig.Slot0.kD = 0;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = DealgaefierConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(DeAlgaefierIOinputs inputs) {
    inputs.armConnected = armMotor.isConnected();
    inputs.armPosition = armMotor.getPosition().getValueAsDouble();
    inputs.armVelocity = armMotor.getVelocity().getValueAsDouble();
    inputs.armAppliedVolts = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armCurrentAmps = armMotor.getStatorCurrent().getValueAsDouble();

    inputs.rollerConnected = rollerMotor.isConnected();
    inputs.rollerPosition = rollerMotor.getPosition().getValueAsDouble();
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();

    Logger.recordOutput(
        "/DeAlgaefier/RollerVelocity", rollerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("/DeAlgaefier/ArmVelocity", armMotor.getVelocity().getValueAsDouble());
  }

  public void runArm(double speed) {
    double desiredRotations = speed * DealgaefierConstants.ARM_PID_POSITION;
    if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadband
      desiredRotations = 0;
    }
    armMotor.setControl(mPositionVoltage.withPosition(desiredRotations));
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void runRoller(double speed) {
    rollerMotor.setControl(mVelocityVoltage.withVelocity(speed));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
