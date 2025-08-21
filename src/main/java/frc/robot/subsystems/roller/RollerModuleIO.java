package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerModuleIO {

  @AutoLog
  public static class RollerModuleInputs {
    public boolean connected = false; 
    public double positionRadius = 0.0;
    public double VelocityRadiusPerSec = 0.0;
    public double appliedVolts = 0.0; //volts applied
    public double Amps = 0.0;
    public RollerState state = RollerState.IDLE; //State of motor
  }

  public default void changeInputs(RollerModuleInputs input) {}
  public default void stopRoller(){}
  public default void Roll(double speed){}
  public default void periodic(){}
}