package frc.robot.subsystems.dealgifier;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
//import frc.robot.subsystems.dealgifier.AlgaeIO.AlgaeInputs;

public class AlgaeSIM implements AlgaeIO{

  //Create Talon FX motor
  private final TalonFX armMotor;
  private final TalonFX rollerMotor;
  //Signals
  private final StatusSignal<Angle> ArmrelativePos;
  private final StatusSignal<AngularVelocity> ArmmotorVelocity;
  private final StatusSignal<Voltage> ArmmotorAppVolts;
  private final StatusSignal<Current> ArmmotorCurr;
  private final Debouncer ArmconnectDebounce = new Debouncer(0.5);

  private final StatusSignal<Angle> RollerRelativePos;
  private final StatusSignal<AngularVelocity> RollermotorVelocity;
  private final StatusSignal<Voltage> RollermotorAppVolts;
  private final StatusSignal<Current> RollermotorCurr;
  private final Debouncer RollerConnectDebounce = new Debouncer(0.5);

  public AlgaeSIM(){
    this.armMotor = new TalonFX(9,"rio");
    this.rollerMotor = new TalonFX(10,"rio");

    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration config2 = new TalonFXConfiguration();
    //Arm Configuration
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 0.5;

    armMotor.getConfigurator().apply(config);
    //Roller Configuration
    config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.CurrentLimits.StatorCurrentLimit = Constants.RollerConstants.ROLLOR_MOTOR_CURRENT_LIMIT;
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 60;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.Slot0.kP = 0.5;

    rollerMotor.getConfigurator().apply(config2);
    //Signal Inititalizations
    ArmrelativePos = armMotor.getPosition();
    ArmmotorVelocity = armMotor.getVelocity();
    ArmmotorAppVolts = armMotor.getMotorVoltage();
    ArmmotorCurr = armMotor.getStatorCurrent();

    RollerRelativePos = rollerMotor.getPosition();
    RollermotorVelocity = rollerMotor.getVelocity();
    RollermotorAppVolts = rollerMotor.getMotorVoltage();
    RollermotorCurr = rollerMotor.getStatorCurrent();


  }
  public void AlgaeChangeInputs(AlgaeInputs inputs){

    //Arm Status refresh
    var ArmStatus = BaseStatusSignal.refreshAll(ArmrelativePos, ArmmotorVelocity, ArmmotorAppVolts, ArmmotorCurr);
            inputs.connected = ArmconnectDebounce.calculate(ArmStatus.isOK());
            inputs.positionRadius = Units.rotationsToRadians(ArmrelativePos.getValueAsDouble());
            inputs.VelocityRadiusPerSec = Units.rotationsToRadians(ArmmotorVelocity.getValueAsDouble());
            inputs.appliedVolts = ArmmotorAppVolts.getValueAsDouble();
            inputs.Amps = ArmmotorCurr.getValueAsDouble();

    //Roller Status refresh
    var RollerStatus = BaseStatusSignal.refreshAll(RollerRelativePos, RollermotorVelocity, RollermotorAppVolts, RollermotorCurr);
            inputs.connected = RollerConnectDebounce.calculate(RollerStatus.isOK());
            inputs.positionRadius = Units.rotationsToRadians(ArmrelativePos.getValueAsDouble());
            inputs.VelocityRadiusPerSec = Units.rotationsToRadians(ArmmotorVelocity.getValueAsDouble());
            inputs.appliedVolts = ArmmotorAppVolts.getValueAsDouble();
            inputs.Amps = ArmmotorCurr.getValueAsDouble();

      //Arm and Roller States
      if (inputs.appliedVolts > 0) {
        inputs.state = AlgaeStates.FORWARD;
      }else if (inputs.appliedVolts < 0) {
        inputs.state = AlgaeStates.REVERSE;
      }else if (inputs.appliedVolts == 0) {
        inputs.state = AlgaeStates.IDLE;
      }else {
        inputs.state = AlgaeStates.UNKNOWN;
      }
  }

  public void Roll(double positioninRotations){
    rollerMotor.set(positioninRotations);
    Logger.recordOutput("/AlgaeRemover/Roller/positionInRotations", positioninRotations);
  }

  public void extendArm(double positionInRotations){
    armMotor.set(positionInRotations);
    Logger.recordOutput("/AlgaeRemover/Arm/positionInRotations", positionInRotations);

  }

  public void stopRoll(){
    rollerMotor.stopMotor();
  }

  public void stopArm(){
    armMotor.stopMotor();
  }
}
