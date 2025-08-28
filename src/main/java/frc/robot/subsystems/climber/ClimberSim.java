package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.ClimberConstants;
//import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;
import frc.robot.util.TunableDouble;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

public class ClimberSim implements ClimberIO{
  private final ThriftyNova motor;
  private TunableDouble revLimit = new TunableDouble("/Climber/config/revLimit", 0);
  private TunableDouble forwardLimit = new TunableDouble("/Climber/config/forwardLimit", 0);
  private TunableDouble kP = new TunableDouble("/Climber/Config/kP", 0.01);
  private TunableDouble kI = new TunableDouble("/Climber/Config/kI", 0.00);
  private TunableDouble kD = new TunableDouble("/Climber/Config/kD", 0.00);
  private TunableDouble kFF = new TunableDouble("/Climber/Config/FF", 0.00);

    public ClimberSim(int id, String bus){
      motor = new ThriftyNova(0);
      motor.setInverted(true);
      motor.setBrakeMode(true);
      motor.setMaxCurrent(CurrentType.STATOR, ClimberConstants.CLIMBER_STATOR);
      motor.setMaxCurrent(CurrentType.SUPPLY, ClimberConstants.CLIMBER_SUPPLY);
      motor.pid0.setP(kP.get());
      motor.pid0.setP(kI.get());
      motor.pid0.setP(kD.get());
      motor.pid0.setP(kFF.get());
      motor.enableSoftLimits(false);
      motor.setSoftLimits(revLimit.get(), forwardLimit.get());
    }

    public void updateInputs(ClimberInputs inputs) {
      inputs.connected = true;
      inputs.positionRad = motor.getPosition();
      inputs.velocityRadPerSec = motor.getVelocity();
      inputs.appliedVolts = motor.getVoltage();
      inputs.currentAmps = motor.getStatorCurrent();
  
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
    public void Climb(double positionInRotations){
      //motor.setPercent(positionInRotations);  
    }
    public void stopClimb(){
      motor.stopMotor();
      Climb(0);
    }
    public void periodic(){

      
      Logger.recordOutput("/Climber/positionInDegrees", motor.getPosition());
      Logger.recordOutput("/Climber/velocityInRotPerSec", motor.getVelocity());
      Logger.recordOutput("/Climber/voltageInVolts", motor.getVoltage());
      Logger.recordOutput("/Climber/statorCurrent", motor.getStatorCurrent());
    }
}
