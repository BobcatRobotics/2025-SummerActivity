package frc.robot.subsystems.dealgifier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

  private final AlgaeIO io;
  private final String name;
  private AlgaeInputsAutoLogged inputs = new AlgaeInputsAutoLogged();

  public AlgaeSubsystem(AlgaeIO io, String name){
    this.io =io;
    this.name = name;
  }
  public void periodic(){
    io.changeInputs(inputs);
    Logger.processInputs(name, inputs);
    
  }
  public void AlgaeRollMotor(double positionInRotations){
    io.AlgaeRollMotor(positionInRotations);
  }
  public void AlgaeExtendArm(double positionInRotations){
    io.AlgaeExtendArm(positionInRotations);
  }
  public void stopArmMotor(){
    io.stopArmMotor();
  }
  public void stopRollerMotor(){
    io.stopRollerMotor();
  }
}


