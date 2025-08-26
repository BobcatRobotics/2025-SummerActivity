package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeIOTalonFX implements AlgaeIO {
  private final TalonFX armMotor;
  private final TalonFX rollerMotor;
  private final VelocityVoltage mVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  public AlgaeIOTalonFX() {
    armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Slot0.kP = 0.11; // TODO tune this
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.ARM_MOTOR_CURRENT_LIMIT;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.ARM_MIN_POSITION;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.ARM_MAX_POSITION;
    armConfig.Feedback.SensorToMechanismRatio = 4;
    armMotor.getConfigurator().apply(armConfig);

    rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = 0.11; // TODO tune this
    rollerConfig.Slot0.kI = 0;
    rollerConfig.Slot0.kD = 0;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    rollerConfig.Feedback.SensorToMechanismRatio = 4;
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

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

    Logger.recordOutput("/Arm/Velocity", armMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("/Arm/TargetVelocityStow", ArmConstants.ARM_ROTATIONS_STOW);
    Logger.recordOutput("/Arm/TargetVelocityDeploy", ArmConstants.ARM_ROTATIONS_DEPLOY);

    Logger.recordOutput("/Roller/Velocity", rollerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("/Roller/TargetVelocityIn", RollerConstants.ROLLER_SPEED_IN);
    Logger.recordOutput("/Roller/TargetVelocityOut", RollerConstants.ROLLER_SPEED_OUT);
  }

  @Override
  public void runArm(double speed) {
    armMotor.set(speed);
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
