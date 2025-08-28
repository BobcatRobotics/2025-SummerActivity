package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public boolean connected = false; 
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0; //volts applied
    public double currentAmps = 0.0;
    public ClimberState state = ClimberState.IDLE; //State of motor
  }
  public default void changeInputs(ClimberInputs inputs) {}
  public default void climb(double positionInRotations) {}
  public default void stopClimber() {}

}