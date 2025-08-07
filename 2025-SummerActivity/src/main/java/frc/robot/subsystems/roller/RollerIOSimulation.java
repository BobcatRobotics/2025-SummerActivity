package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class RollerIOSimulation implements RollerIO {
  final TalonFX rollerMotor = new TalonFX(0, "rio");
  VelocityVoltage rollerMotorRequest = new VelocityVoltage(0).withSlot(0);

  private double targetVelocityRadPerSec = 0.0;

  // Simulation
  private final TalonFXSimState simState;

  private double simulatedPosition = 0.0;
  private double simulatedVelocity = 0.0;
  private double maxSimVelocity = 100.0; // rotations/sec
  private double maxAcceleration = 1000.0; // RPS * RPS
  private final double simLoopPeriodSec = 0.02; // 20ms typical loop time

  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.3;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.045; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim driveSim;
  private boolean driveClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;

  public RollerIOSimulation() {
    var talonFXConfigurator = rollerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();
    var feedbackConfigs = new FeedbackConfigs();

    feedbackConfigs.SensorToMechanismRatio = 25;

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to counter-clockwise
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator.apply(motorConfigs);
    talonFXConfigurator.apply(feedbackConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    rollerMotor.getConfigurator().apply(slot0Configs);

    simState = rollerMotor.getSimState();

    driveSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, 0.001, 25), DRIVE_GEARBOX);
  }

  public void updateInputs(RollerIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);

    // Update drive inputs
    inputs.velocity = driveSim.getAngularVelocityRadPerSec();
    Logger.recordOutput("/Roller/Velocity", inputs.velocity);
    Logger.recordOutput("/Roller/driveTarget", targetVelocityRadPerSec);
  }

  public void setSpeed(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
    this.setSpeed(0);
  }
}
