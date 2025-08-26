package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
  private ClimberIO io;
  private ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
  private final String name;

  public ClimberSubsystem(ClimberIO io, String name){
      this.io = io;
      this.name = name;
  }
  public void periodic(){
    io.changeInputs(inputs);
    Logger.processInputs(name, inputs);
  }
  public void Climb(double positionInRotations) {
    io.Climb(positionInRotations);
  }
  public void stopClimber(double positionInRotations) {
    io.stopClimber();
  }
}
