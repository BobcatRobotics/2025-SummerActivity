package frc.robot.subsystems.drive.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberModuleIO {
  @AutoLog
  public static class ClimberModuleIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadperSec = 0.0;
    public double appliedVolts = 0.0;
    public double Amps = 0.0;
    // all of that was setting up the stuff to be logged

    public ClimberState state = ClimberState.IDLE;
  }

  public default void updateInputs(ClimberModuleIOInputs inputs) {}

  public default void runClimber(double positionInRotations) {}
  ;

  public default void stopClimber() {}

  public void periodic();
  ;
}
