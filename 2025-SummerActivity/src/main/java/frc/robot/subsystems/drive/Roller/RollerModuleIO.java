package frc.robot.subsystems.drive.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerModuleIO {

  @AutoLog
  public static class RollerModuleIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double amps = 0.0;
    // all of that was setting up the stuff to be logged

    public RollerState state = RollerState.IDLE;
  }

  public default void updateInputs(RollerModuleIOInputs inputs) {}

  public default void runRoller(double speedInRadians) {}
  ;

  public default void stopRoller() {}
  ;
}
