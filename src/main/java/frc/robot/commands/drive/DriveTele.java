// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain.Drive;


public class DriveTele extends Command {
  /** Creates a new DriveTele. */
  private Drive drive;

  private double modifyInputs(double val, boolean isRot){
    if (isRot){
      if (Math.abs(val)<drive.getAngDeadband()){
        val =0;
      }
      return val*drive.getAng();
    }
    else{
      if (Math.abs(val)<drive.getTanDeadband()){
        val =0;
      }
      return val*drive.getTan();
    }
  }

  public void driveFromChassis(ChassisSpeeds speeds){
    var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states,DriveConstants.MAX_TANGENTIAL_VELOCITY);
    drive.setModuleStates(states);
  }

  private DoubleSupplier x,y,z;


  public DriveTele(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive instance) {
    this.x = fwd;
    this.y = str;
    this.z = rot;

    drive = instance;
    addRequirements(drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(modifyInputs(x.getAsDouble(), false), modifyInputs(y.getAsDouble(), false),modifyInputs(z.getAsDouble(), true),
    drive.getDriveHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveFromChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
