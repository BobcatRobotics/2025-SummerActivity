package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
//import org.littletonrobotics.junction.Logger;
//import com.ctre.phoenix6.configs.Slot0Configs;
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

public class ArmModuleReal implements ArmModuleIO {

    private final TalonFX motor;
    private final StatusSignal<Angle> relativePos;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppVolts;
    private final StatusSignal<Current> motorCurr;
    private final Debouncer connectDebounce = new Debouncer(0.5);
    private final TalonFXConfiguration config;
  

    public ArmModuleReal(int id, String bus){
      this.motor = new TalonFX(9,"rio");
      config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.CurrentLimits.StatorCurrentLimit = 60;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.CurrentLimits.SupplyCurrentLimit = 60;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      motor.getConfigurator().apply(config, 0.25);

      relativePos = motor.getPosition();
      motorVelocity = motor.getVelocity();
      motorAppVolts = motor.getMotorVoltage();
      motorCurr = motor.getStatorCurrent();
    }
    
    public void changeInputs(ArmModuleInputs input){
      var motorStatus = 
        BaseStatusSignal.refreshAll(
            relativePos, motorVelocity, motorAppVolts, motorCurr);

            input.connected = connectDebounce.calculate(motorStatus.isOK());
            input.positionRadius = Units.rotationsToRadians(relativePos.getValueAsDouble());
            input.VelocityRadiusPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
            input.appliedVolts = motorAppVolts.getValueAsDouble();
            input.Amps = motorCurr.getValueAsDouble();

      if (input.appliedVolts > 0) {
        input.state = ArmState.FORWARD;
      } 
      else if (input.appliedVolts < 0) {
        input.state = ArmState.REVERSE;
      } 
      else if (input.appliedVolts == 0) {
        input.state = ArmState.IDLE;
      } 
      else {
        input.state = ArmState.UNKNOWN;
      }
    }

    public void runArm(double positionInRotations){
      motor.set(positionInRotations);
      Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
    }
    public void stopArm(){

    }
}
