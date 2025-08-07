package frc.robot.subsystems.roller;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RollerModuleIOInputsAutoLogged extends RollerModuleIO.RollerModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PositionRad", positionRad);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentAmps", currentAmps);
    table.put("State", state);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    positionRad = table.get("PositionRad", positionRad);
    velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentAmps = table.get("CurrentAmps", currentAmps);
    state = table.get("State", state);
  }

  public RollerModuleIOInputsAutoLogged clone() {
    RollerModuleIOInputsAutoLogged copy = new RollerModuleIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.positionRad = this.positionRad;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentAmps = this.currentAmps;
    copy.state = this.state;
    return copy;
  }
}
