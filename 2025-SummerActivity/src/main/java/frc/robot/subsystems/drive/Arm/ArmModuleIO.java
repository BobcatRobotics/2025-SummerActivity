package frc.robot.subsystems.drive.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmModuleIO {

  @AutoLog
  public static class ArmModuleIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadperSec = 0.0;
    public double appliedVolts = 0.0;
    public double Amps = 0.0;
    // all of that was setting up the stuff to be logged

    public ArmState state = ArmState.IDLE;
  }

  public default void updateInputs(ArmModuleIOInputs inputs) {}

  public default void runArm(double positionInRotations) {}
  ;

  public default void stopArm() {}
  ;
}
