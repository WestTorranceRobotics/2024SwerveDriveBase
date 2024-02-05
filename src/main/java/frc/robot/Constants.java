// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants{
    public static final double MAX_TANGENTIAL_VELOCITY = 1.2; 
    public static final double MAX_TELE_TANGENTIAL_VELOCITY = 1.5; 
    public static final double MAX_TELE_ANGULAR_VELOCITY = 2 * Math.PI; 
    public static final double WHEELBASE_WIDTH =  Units.inchesToMeters(27);
    public static final double WHEELBASE_LENGTH = Units.inchesToMeters(27);
    public static final double WHEEL_DIAMETER_METERS = 0.1016;


    public static final int REAR_RIGHT_DRIVE_PORT = 28;
    public static final int FRONT_RIGHT_DRIVE_PORT = 25;
    public static final int FRONT_LEFT_DRIVE_PORT = 27;
    public static final int REAR_LEFT_DRIVE_PORT = 26;

    public static final int REAR_RIGHT_TURN_PORT = 12;
    public static final int FRONT_RIGHT_TURN_PORT = 8;
    public static final int FRONT_LEFT_TURN_PORT = 10;
    public static final int REAR_LEFT_TURN_PORT = 6;

    public static final int REAR_RIGHT_ENCODER_PORT = 16;
    public static final int FRONT_RIGHT_ENCODER_PORT = 17;
    public static final int FRONT_LEFT_ENCODER_PORT = 15;
    public static final int REAR_LEFT_ENCODER_PORT = 18;

    public static final double REAR_RIGHT_ENCODER_OFFSET = -141.67;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = -119.17;
    public static final double FRONT_LEFT_ENCODER_OFFSET = 152.25;
    public static final double REAR_LEFT_ENCODER_OFFSET = 38.75;


    public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);


    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION,FRONT_RIGHT_MODULE_POSITION,REAR_LEFT_MODULE_POSITION,REAR_RIGHT_MODULE_POSITION);





    public static final double kWheelDiameterMeters = 0.1016;
    public static final double ticksPerWheelRotation = 4056; //Change here later :))
    public static final double l3Ratio = (1/6.12);

    public static final double metersPerWheelRotation =  (l3Ratio)*Math.PI*(kWheelDiameterMeters); //Change here later :))


    public static final double kDriveMotorGearRatio = 1.0/2048;
    // * kWheelDiameterMeter  

    public static final double TURN_kP = 0.0075;
    public static final double TURN_kD = 0;

    public static final double AUTO_CONTROLLER_kP = 2;

    public static final SwerveModuleState[] LOCKED_MODULE_STATES = {new SwerveModuleState(0, Rotation2d.fromDegrees(45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(45))};

    public static final double MAX_DRIVE_TANGENTIAL_ACCEL = 4; // in units of m/s/s

  }
}
