package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public class AlgaeIOInputs {
    public double armSpeed = 0.00;
    public double rollerSpeed = 0.00;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setArmSpeed(double armSpeed) {}

  public default void setRollerSpeed(double rollerSpeed) {}

  public default void stopRoller() {}

  public default void stopArm() {}
}
