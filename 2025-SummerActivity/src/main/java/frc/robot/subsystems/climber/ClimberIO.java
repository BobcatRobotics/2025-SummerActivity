package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double speed = 0.00;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void stopClimber() {}
}
