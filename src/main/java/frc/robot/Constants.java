// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static class DriveConstants {
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    public static final double kErrorBound = 0;

    public static final double kTanDeadband = 0.15;
    public static final double kAngDeadband = 0.15;
 

  }

  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    // public static final double kDriveDistanceLoss = 0.95; // from measuring IRL
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final double kPTurning = 0.0075; // 0.6
    public static final double kITurning = 0; 
    public static final double kDTurning = 0; 
    public static final double kFTurning = 0; 

    public static final double kPDrive = 0.125; // 0.6
    public static final double kIDrive = 0;
    public static final double kDDrive = 0.01; 
    public static final double kVDrive = 0.11; 

    public static final String kCANivoreName = "rio";



  } 

  public static final class SwerveDriveConstants {

  // Distance between
    private double tanDeadband = 0.15;
    private double angDeadband = 0.15;
    public static final double kTrackWidth = 0.635;
    // Distance between front and back wheels
    public static final double kWheelBase = 0.635;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2));
    
    public static final int kFRDriveID = 25;
    public static final int kFLDriveID = 27;
    public static final int kBLDriveID = 26;
    public static final int kBRDriveID = 28;

    public static final int kFRTurningID = 8;
    public static final int kFLTurningID = 10;
    public static final int kBLTurningID = 6;
    public static final int kBRTurningID = 12;

    public static final boolean kFRTurningReversed = false;
    public static final boolean kFLTurningReversed = false; 
    public static final boolean kBLTurningReversed = false; 
    public static final boolean kBRTurningReversed = false; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

  

   public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 17;
      public static final int kFLCANCoderID = 15;
      public static final int kBLCANCoderID = 18;
      public static final int kBRCANCoderID = 16;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false;
      
      // public static final double kBRCANCoderOffset = -141.67;
      // public static final double kFRCANCoderOffset = -119.17;
      // public static final double kFLCANCoderOffset= 152.25;
      // public static final double kBLCANCoderOffset = 38.75;

      public static final double kFLEncoderOffset = 119.6+90;
      public static final double kBLEncoderOffset = -130.7+90;
      public static final double kFREncoderOffset = 33.1+90;
      public static final double kBREncoderOffset = 52.82+90;
  
      
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    //TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -3; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(315))
    };

    public static final double kGravityMPS = 9.80665;
  } 

  public static final class SwerveAutoConstants {
    public static final double kPTurnToAngle = SmartDashboard.getNumber("kP Theta Teleop", 6);
    public static final double kITurnToAngle = SmartDashboard.getNumber("kI Theta Teleop", 0);
    public static final double kDTurnToAngle = SmartDashboard.getNumber("kD Theta Teleop", 0.2);
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;


  public static final class PathPlannerConstants {
    public static final double kPPMaxVelocity = 3;
    public static final double kPPMaxAcceleration = 3;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;
    public static final PathConstraints kPPPathConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration, 
                                                                                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

    public static final double kPP_P = 0;
    public static final double kPP_I = 0;
    public static final double kPP_D = 0;
    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 0.25;
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0;
    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;
  }
}
}