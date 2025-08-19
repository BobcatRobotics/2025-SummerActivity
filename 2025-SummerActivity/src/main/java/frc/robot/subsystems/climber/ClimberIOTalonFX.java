package frc.robot.subsystems.climber;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberIOTalonFX {
  private final ThriftyNova climberMotor;

  public ClimberIOTalonFX() {
    climberMotor = new ThriftyNova(ClimberConstants.CLIMBER_MOTOR_ID);
    climberMotor.setInverted(true);
    climberMotor.setBrakeMode(true);
    climberMotor.setMaxCurrent(CurrentType.STATOR, ClimberConstants.CLIMBER_STATOR);
    climberMotor.setMaxCurrent(CurrentType.SUPPLY, ClimberConstants.CLIMBER_SUPPLY);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberPosition = climberMotor.getPosition();
    inputs.climberVelocity = climberMotor.getVelocity();
    inputs.climberAppliedVolts = climberMotor.getVoltage();
    inputs.climberCurrentAmps = climberMotor.getStatorCurrent();

    Logger.recordOutput("/Climber/Velocity", climberMotor.getVelocity());
    Logger.recordOutput("/Climber/TargetVelocityIn", ClimberConstants.CLIMBER_SPEED_IN);
    Logger.recordOutput("/Climber/TargetVelocityOut", ClimberConstants.CLIMBER_SPEED_OUT);
  }

  public void runClimber(double speed) {
    climberMotor.setPercent(speed);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }
}
