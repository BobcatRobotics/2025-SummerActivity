package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;

public class RollerIOTalonFX implements RollerIO {
  final TalonFX rollerMotor = new TalonFX(10, "rio");
  VelocityVoltage rollerMotorRequest = new VelocityVoltage(0).withSlot(0);

  public RollerIOTalonFX() {
    var talonFXConfigurator = rollerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to counter-clockwise
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator.apply(motorConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    rollerMotor.getConfigurator().apply(slot0Configs);
  }

  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = rollerMotor.getVelocity().getValueAsDouble();
    Logger.recordOutput("/Roller/Velocity", inputs.velocity);
  }

  public void setSpeed(double speed) {

    rollerMotor.setControl(rollerMotorRequest.withVelocity(speed).withFeedForward(0.5));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
