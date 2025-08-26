package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ClimberIOTalonFX implements ClimberIO {
  final ThriftyNova climberMotor = new ThriftyNova(1);
  // create a position closed-loop request, voltage output, slot 0 configs
  final PositionVoltage armMotorRequest = new PositionVoltage(0).withSlot(0);

  public ClimberIOTalonFX() {
    // enable stator current limit
    climberMotor.setMaxCurrent(CurrentType.STATOR, 20);

    // set invert mode
    climberMotor.setInverted(true);

    // set brake mode
    climberMotor.setBrakeMode(true);

    climberMotor.enableSoftLimits(false);
    climberMotor.setSoftLimits(0, 0);
  }

  public void updateInputs(ArmIOInputs inputs) {}

  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }
}
