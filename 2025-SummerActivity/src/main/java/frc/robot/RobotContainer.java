// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import java.util.Map;
=======
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
>>>>>>> Stashed changes

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< Updated upstream
    // The enum used as keys for selecting the command to run.
    private enum CommandSelector {
      ONE,
      TWO,
      THREE
    }

    /*
     * An example selector method for the selectcommand. Returns the selector that
     * will select
     * which command to run. Can base this choice on logical conditions evaluated at
     * runtime.
     */
    private CommandSelector select() {
        return CommandSelector.ONE;
    }
=======
  // Subsystems
  private final Drive drive;
  private final RollerSubsystem roller;

  // Commands
  public Command rollerInCommand;
  public Command rollerStopCommand;
  public Command rollerSlowOutCommand;
  public Command rollerFastOutCommand;

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
        break;
    }

    // Set up the remaining subsystems
    roller = new RollerSubsystem();
    rollerInCommand =
        new InstantCommand(
            () -> roller.runRoller(Constants.RollerConstants.ROLLER_SPEED_IN));
    rollerStopCommand = Commands.runOnce(() -> roller.stopRoller(), roller);
    rollerSlowOutCommand =
        new InstantCommand(
            () -> roller.runRoller(Constants.RollerConstants.ROLLER_SLOW_SPEED_OUT));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
>>>>>>> Stashed changes

    /*
     * An example selectcommand. Will select from the three commands based on the
     * value returned
     * by the selector method at runtime. Note that selectcommand works on Object(),
     * so the
     * selector does not have to be an enum; it could be any desired type (string,
     * integer,
     * boolean, double...)
     */
    private final Command exampleSelectCommand = new SelectCommand<>(
        // Maps selector values to commands
        Map.ofEntries(
        Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
        Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
        Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!"))),
        this::select);

    public RobotContainer() {
      // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

<<<<<<< Updated upstream
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return exampleSelectCommand;
    }
=======
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.y().whileTrue(rollerInCommand).onFalse(rollerStopCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
>>>>>>> Stashed changes
}
