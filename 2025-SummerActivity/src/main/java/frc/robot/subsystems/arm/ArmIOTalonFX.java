package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor;
  private final PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);

  public ArmIOTalonFX(int motorID) {
    armMotor = new TalonFX(motorID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ArmConstants.ARM_MOTOR_CURRENT_LIMIT;
    config.Feedback.SensorToMechanismRatio = 4;

    // Velocity PIDs
    config.Slot0.kP = 0.11; // TODO tune this
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    armMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ArmIOinputs inputs) {
    inputs.armConnected = armMotor.isConnected();
    inputs.armPosition = armMotor.getPosition().getValueAsDouble();
    inputs.armVelocity = armMotor.getVelocity().getValueAsDouble();
    inputs.armAppliedVolts = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armCurrentAmps = armMotor.getSupplyCurrent().getValueAsDouble();

    Logger.recordOutput("/Arm/Velocity", armMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("/Arm/TargetVelocityStow", ArmConstants.ARM_ROTATIONS_STOW);
    Logger.recordOutput("/Arm/TargetVelocityDeploy", ArmConstants.ARM_ROTATIONS_DEPLOY);
  }

  public void positionArm(double pos) {
    double desiredRotations = pos * ArmConstants.ARM_PID_POSITION;
    if (Math.abs(desiredRotations) <= 0.1) {
      desiredRotations = 0;
    }
    armMotor.setControl(mPositionVoltage.withPosition(desiredRotations));
  }

  public void stopArm() {
    armMotor.stopMotor();
  }
}
