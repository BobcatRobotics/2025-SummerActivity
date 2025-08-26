package frc.robot.subsystems.algaeRemover;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class AlgaeRemoverIOTalonFX implements AlgaeRemoverIO {
  final TalonFX algaeRemoverPositionMotor = new TalonFX(11, "rio");
  final TalonFX algaeRemoverWheelMotor = new TalonFX(12, "rio");
  // create a velocity closed-loop request, voltage output, slot 1 configs
  final VelocityVoltage wheelMotorRequest = new VelocityVoltage(0).withSlot(0);

  public AlgaeRemoverIOTalonFX() {
    var algaeRemoverPositionTalonFXConfigurator = algaeRemoverPositionMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to clockwise
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    algaeRemoverPositionTalonFXConfigurator.apply(limitConfigs);
    algaeRemoverPositionTalonFXConfigurator.apply(motorConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    algaeRemoverPositionMotor.getConfigurator().apply(slot0Configs);

    var algaeRemoverWheelTalonFXConfigurator = algaeRemoverWheelMotor.getConfigurator();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to clockwise
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    algaeRemoverWheelTalonFXConfigurator.apply(limitConfigs);
    algaeRemoverWheelTalonFXConfigurator.apply(motorConfigs);

    var slot1Configs = new Slot1Configs();
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    algaeRemoverWheelMotor.getConfigurator().apply(slot1Configs);
  }

  public void updateInputs(ArmIOInputs inputs) {}

  public void setPosition(double position) {
    algaeRemoverPositionMotor.set(position);
  }

  public void setSpeed(double speed) {
    algaeRemoverWheelMotor.setControl(wheelMotorRequest.withVelocity(speed));
  }

  public void stopPositionMotor() {
    algaeRemoverPositionMotor.stopMotor();
  }

  public void stopWheelMotor() {
    algaeRemoverWheelMotor.stopMotor();
  }
}
