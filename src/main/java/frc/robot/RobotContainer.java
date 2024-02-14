// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.GyroStuffs.PigeonV2;
import frc.robot.subsystems.LEDs.DriveTrainLEDs;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

import javax.print.attribute.standard.NumberUp;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  PS5Controller driveController = new PS5Controller(0);
  private JoystickButton squareButton = new JoystickButton(driveController, 1);
  private PigeonV2 gyro = new PigeonV2(0);
  private SwerveDriveTrain drive = new SwerveDriveTrain(gyro);
  private DriveTrainLEDs dtLEDs = new DriveTrainLEDs();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new SwerveJoystickCommand(driveController::getLeftY, driveController::getLeftX, driveController::getRightX, drive));
    // Configure the trigger bindings
    configureBindings();
    initShuffleboard();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }
  public void initShuffleboard() {
  drive.initModuleShuffleboard(1);
  drive.initMainShuffleboard(1);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");

    // An example command will be run in autonomous
   // return null;
  }
}