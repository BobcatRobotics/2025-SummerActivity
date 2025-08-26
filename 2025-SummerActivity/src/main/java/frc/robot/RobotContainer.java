// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algaeRemover.AlgaeRemover;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Vision vision;

  // Subsystems
  private final Drive drive;
  // private final Roller roller = new Roller();
  // private final Arm arm = new Arm();
  private final AlgaeRemover algaeRemover = new AlgaeRemover();
  private final Climber climber = new Climber();
  private final Algae algae = new Algae();

  private final Command setRollerFullSpeed;
  private final Command setRollerReverseFullSpeed;
  private final Command stopRoller;
  private final Command armIn;
  private final Command armOut;
  private final Command stopArm;
  private final Command algaeRemoverForward;
  private final Command algaeRemoverBackward;
  private final Command algaeRemoverFullSpeed;
  private final Command algaeRemoverReverseFullSpeed;
  private final Command stopPositionMotor;
  private final Command stopWheelMotor;
  private final Command climberIn;
  private final Command climberOut;
  private final Command stopClimber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(drive::addVisionMeasurement, new VisionIOLimelight("", drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    setRollerFullSpeed = new RunCommand(() -> algae.setRollerSpeed(25), algae);
    setRollerReverseFullSpeed = new RunCommand(() -> algae.setRollerSpeed(-25), algae);
    stopRoller = new InstantCommand(() -> algae.stopRoller());
    armIn = new RunCommand(() -> algae.setArmSpeed(-.19));
    armOut = new RunCommand(() -> algae.setArmSpeed(.177));
    stopArm = new InstantCommand(() -> algae.stopArm());
    algaeRemoverFullSpeed = new RunCommand(() -> algaeRemover.setSpeed(10), algaeRemover);
    algaeRemoverReverseFullSpeed = new RunCommand(() -> algaeRemover.setSpeed(-10), algaeRemover);
    algaeRemoverForward = new RunCommand(() -> algaeRemover.setPosition(0.19), algaeRemover);
    algaeRemoverBackward = new RunCommand(() -> algaeRemover.setPosition(-0.19), algaeRemover);
    stopPositionMotor = new InstantCommand(() -> algaeRemover.stopPositionMotor(), algaeRemover);
    stopWheelMotor = new InstantCommand(() -> algaeRemover.stopWheelMotor(), algaeRemover);
    climberIn = new RunCommand(() -> climber.setSpeed(0.6), climber);
    climberOut = new RunCommand(() -> climber.setSpeed(-0.6), climber);
    stopClimber = new RunCommand(() -> climber.stopClimber(), climber);

    // Named command for auto
    NamedCommands.registerCommand("scoreCoral", setRollerFullSpeed);
    NamedCommands.registerCommand("stopScoring", stopRoller);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    /*
    // Lock to 0 when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    //controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    */
    // Run roller forward when right trigger is pressed
    controller.rightTrigger().whileTrue(setRollerFullSpeed).onFalse(stopRoller);

    // Run roller backward when left trigger is pressed
    controller.leftTrigger().whileTrue(setRollerReverseFullSpeed).onFalse(stopRoller);

    // Run arm forward when right bumper is pressed
    controller.rightBumper().whileTrue(armIn).onFalse(stopArm);

    // Run arm backward when left bumper is pressed
    controller.leftBumper().whileTrue(armOut).onFalse(stopArm);

    // Run algae remover wheel forward
    controller.y().whileTrue(algaeRemoverFullSpeed).onFalse(stopWheelMotor);

    // Run algae remover wheel backward
    controller.x().whileTrue(algaeRemoverReverseFullSpeed).onFalse(stopWheelMotor);

    // Run algae remover position forward
    controller.a().whileTrue(algaeRemoverForward).onFalse(stopPositionMotor);

    // Run algae remover position backward
    controller.b().whileTrue(algaeRemoverBackward).onFalse(stopPositionMotor);

    // Run climber in
    controller.povUp().whileTrue(climberIn).onFalse(stopClimber);

    // Run climber out
    controller.povDown().whileTrue(climberOut).onFalse(stopClimber);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
