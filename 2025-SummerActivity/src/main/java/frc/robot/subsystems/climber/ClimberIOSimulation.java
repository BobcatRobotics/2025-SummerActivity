package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ClimberIOSimulation implements ClimberIO {
  final ThriftyNova climberMotor = new ThriftyNova(0);
  // create a position closed-loop request, voltage output, slot 0 configs
  final PositionVoltage armMotorRequest = new PositionVoltage(0).withSlot(0);

  public ClimberIOSimulation() {
    // enable stator current limit
    climberMotor.setMaxCurrent(CurrentType.STATOR, 20);

    // set invert mode
    climberMotor.setInverted(true);

    // set brake mode
    climberMotor.setBrakeMode(true);
  }

  public void updateInputs(ArmIOInputs inputs) {}

  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }
}
