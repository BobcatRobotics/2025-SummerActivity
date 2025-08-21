package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOTalonFX implements ArmIO {
  final TalonFX armMotor = new TalonFX(9, "rio");
  // create a position closed-loop request, voltage output, slot 0 configs
  final PositionVoltage armMotorRequest = new PositionVoltage(0).withSlot(0);

  public ArmIOTalonFX() {
    var talonFXConfigurator = armMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to clockwise
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator.apply(motorConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 0; // To overcome gravity
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    armMotor.getConfigurator().apply(slot0Configs);
  }

  public void updateInputs(ArmIOInputs inputs) {}

  public void setPosition(double position) {
    armMotor.set(position);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }
}
