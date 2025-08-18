package frc.robot.subsystems.drive.AlgaeRemover;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRemoverIO  {

    @AutoLog
    public static class AlgaeRemoverIOInputs {
      public boolean connected = false;
      public double armPositionRad = 0.0;
      public double armVelocityRadPerSec = 0.0;
      public double armVolts = 0.0;
      public double armAmps = 0.0;
  
      public double rollerPositionRad = 0.0;
      public double rollerVelocityRadPerSec = 0.0;
      public double rollerVolts = 0.0;
      public double rollerAmps = 0.0;
  
      public AlgaeRemoverRollerState rollerState = AlgaeRemoverRollerState.IDLE;
      public AlgaeRemoverArmState armState = AlgaeRemoverArmState.IDLE;
    }
  
    public default void updateInputs(AlgaeRemoverIOInputs inputs) {}
  
    public default void runArm(double positionInRotations) {}
    public default void runRoller(double positionInRotations) {}
    ;
  
    public default void stopArm() {}
    public default void stopRoller(){}
    ;
  }