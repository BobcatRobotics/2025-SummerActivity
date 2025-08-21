package frc.robot.subsystems.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
//import frc.robot.util.TunableDouble;
import frc.robot.util.TunableDouble;

public class RollerModuleSim implements RollerModuleIO {

    private final TalonFX motor;
    private final StatusSignal<Angle> relativePos;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppVolts;
    private final StatusSignal<Current> motorCurrent;
    //private final Debouncer connectDebounce = new Debouncer(0.5);
    private final TalonFXConfiguration config;

    //private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    //private final PositionVoltage positionControl = new PositionVoltage(0);
    /*     
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final PIDController simPID = new PIDController(0.0, 0.0, 0.00);
    private static final DCMotor dcMotor = DCMotor.getKrakenX60(1);
    private static final double gear_ratio = 25.0;
    private static final double MOI = 0.0009;
    
    private final TunableDouble kI = new TunableDouble("/Roller/Config/kI", 0.0);
    private final TunableDouble kD = new TunableDouble("/Roller/Config/kD", 0.0);
    private final TunableDouble kV = new TunableDouble("/Roller/Config/kV", 0.0);
    private TalonFXSimState motorSim;

    private double simposition = 0.0;
    private double simVelocity = 0.0;
    private double targetVelocity = 0.0;
    private double maxSimVelocity = 10.0;
    private double maxAcceleration = 100.0;
    private final double simLoopPeriod = 0.02;*/


    private final TunableDouble kP = new TunableDouble("/Roller/Config/kP", 0.1);


    public RollerModuleSim(int id, String bus){
      this.motor = new TalonFX(9,"rio");

      config = new TalonFXConfiguration();
      config.Feedback.SensorToMechanismRatio = 75;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLOR_MOTOR_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kP.get();
      config.Slot0.kD = kP.get();
      config.Slot0.kV = kP.get();

      relativePos = motor.getPosition();
      motorVelocity = motor.getVelocity();
      motorAppVolts = motor.getMotorVoltage();
      motorCurrent = motor.getStatorCurrent();

       BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, relativePos, motorVelocity, motorAppVolts, motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
/* public void updateTunnables() {
      if (kP.check()) {
        config.Slot0.kP = kP.get();
      }
      if (kI.check()) {
        config.Slot0.kI = kI.get();
      }
      if (kP.check()) {
        config.Slot0.kD = kD.get();
      }
      if (kP.check()) {
        config.Slot0.kV = kV.get();
      }
      motor.getConfigurator().apply(config, 0.25);
    }
    motor.getConfigurator().apply(config, 0.25);*/
    

    }
 

    public void Roll(){

    }
    public void stopRoller(){
      motor.stopMotor();
    }
    public void periodic(){

    }
}


