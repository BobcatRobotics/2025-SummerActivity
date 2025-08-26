package frc.robot.subsystems.drive.Climber;

import org.littletonrobotics.junction.Logger;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import frc.robot.Constants.ClimberConstants;
import frc.robot.util.TunableDouble;

public class ClimberModuleSim implements ClimberModuleIO {

  private final ThriftyNova motor;
  private TunableDouble kP = new TunableDouble("/Climber/config/kP", 0.01);
  private TunableDouble kI = new TunableDouble("/Climber/config/kI", 0.00);
  private TunableDouble kD = new TunableDouble("/Climber/config/kD", 0.00);
  private TunableDouble kFF = new TunableDouble("/Climber/onfig/FF", 0.00);
  private TunableDouble reverseLimit = new TunableDouble("/Climber/config/reverseLimit", 0);
  private TunableDouble forwardLimit = new TunableDouble("/Climber/config/forwardLimit", 0);

  public ClimberModuleSim(int id, String bus) {
    motor = new ThriftyNova(ClimberConstants.CLIMBER_MOTOR_ID);
    motor.setInverted(true);
    motor.setBrakeMode(true);
    motor.setMaxCurrent(CurrentType.STATOR, ClimberConstants.CLIMBER_STATOR);
    motor.setMaxCurrent(CurrentType.SUPPLY, ClimberConstants.CLIMBER_SUPPLY);
    motor.pid0.setP(kP.get());
    motor.pid0.setI(kI.get());
    motor.pid0.setD(kD.get());
    motor.pid0.setFF(kFF.get());
    motor.enableSoftLimits(true);
    motor.setSoftLimits(reverseLimit.get(), forwardLimit.get());
  }

  @Override 
  public void updateInputs(ClimberModuleIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRad = motor.getPosition();
    inputs.velocityRadperSec = motor.getVelocity();
    inputs.appliedVolts = motor.getVoltage();
    inputs.Amps = motor.getStatorCurrent();

    if (inputs.appliedVolts > 0) {
      inputs.state = ClimberState.FORWARD;
    } else if (inputs.appliedVolts < 0) {
      inputs.state = ClimberState.REVERSE;
    } else if (inputs.appliedVolts == 0) {
      inputs.state = ClimberState.IDLE;
    } else {
      inputs.state = ClimberState.UNKNOWN;
    }
  }

  public void runClimber(double positionInDegrees) {
    motor.setPosition(positionInDegrees);
  }

  public void stopClimber() {
    motor.stopMotor();
    runClimber(0);
  }

  public void periodic() {
    if (kP.check()) {
      motor.pid0.setP(kP.get());
    }
    if (kI.check()) {
      motor.pid0.setI(kI.get());
    }
    if (kD.check()) {
      motor.pid0.setD(kD.get());
    }
    if (kFF.check()) {
      motor.pid0.setFF(kFF.get());
    }
    if (reverseLimit.check() || forwardLimit.check()) {
      motor.setSoftLimits(reverseLimit.get(), forwardLimit.get());
    }
    
    Logger.recordOutput("/Climber/positionInDegrees", motor.getPosition());
    Logger.recordOutput("/Climber/velocityInRotPerSec", motor.getVelocity());
    Logger.recordOutput("/Climber/voltageInVolts", motor.getVoltage());
    Logger.recordOutput("/Climber/statorCurrent", motor.getStatorCurrent());
  }
}