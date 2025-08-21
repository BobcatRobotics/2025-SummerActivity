package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.roller.RollerIO.RollerIOInputs;

public class AlgaeIOSimulation implements AlgaeIO {
  final TalonFX armMotor = new TalonFX(9, "rio");
  final TalonFX rollerMotor = new TalonFX(10, "rio");
  VelocityVoltage rollerMotorRequest = new VelocityVoltage(0).withSlot(0);

  public AlgaeIOSimulation() {
    var armTalonFXConfigurator = armMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to clockwise
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    armTalonFXConfigurator.apply(limitConfigs);
    armTalonFXConfigurator.apply(motorConfigs);

    var rollerTalonFXConfigurator = rollerMotor.getConfigurator();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to counter-clockwise
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    rollerTalonFXConfigurator.apply(limitConfigs);
    rollerTalonFXConfigurator.apply(motorConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    rollerMotor.getConfigurator().apply(slot0Configs);
  }

  public void updateInputs(RollerIOInputs inputs) {}

  public void setArmSpeed(double armSpeed) {
    armMotor.set(armSpeed);
  }

  public void setRollerSpeed(double rollerSpeed) {
    armMotor.set(rollerSpeed);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
