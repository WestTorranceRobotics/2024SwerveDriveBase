// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.security.cert.TrustAnchor;

import org.opencv.core.Mat;
import org.opencv.video.Tracker;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */


  ShuffleboardTab troubleshooShuffleboardTab = Shuffleboard.getTab("TroubleShooting");
  private GenericEntry SBInputKp = troubleshooShuffleboardTab.add("KP", 0).withPosition(0, 0).getEntry();
    private GenericEntry SBEnableChangePID = troubleshooShuffleboardTab.add("Change PID", 0).withPosition(0, 1).getEntry();


  private double driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY; //constants here
  private double driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY; //constants here
  private double tanDeadband = 0.15;
  private double angDeadband = 0.15;

  private Alliance lastKnownAlliance;

  private static Drive instance = new Drive();
  private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
  private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
  private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
  private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
  
 // private AHRS gyro = new AHRS(Port.kMXP);

  private Pigeon2 gyro = new Pigeon2(0);
  private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI, Math.PI);
  private ProfiledPIDController rotController = new ProfiledPIDController(0.5, 0, 0, rotProfile);

  private Pose2d robotPosition;

  private SwerveDriveOdometry swerveDriveOdometry;
  SwerveDriveKinematics kinematics;
  


  public Drive() {

    swerveDriveOdometry = new SwerveDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, getDriveHeading(), getModulePositions());

    kinematics = DriveConstants.DRIVE_KINEMATICS;




    rotController.setTolerance(5);
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    gyro.reset();


    SmartDashboard.putNumber("align P", 0.25);
    SmartDashboard.putNumber("strafe P", 0.25);
    SmartDashboard.putNumber("offset strafe", 0);


        
    // All other subsystem initialization
        // ...

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

  public static Drive getInstance(){
    if(instance ==null){
      return new Drive();
    }
    else
      return instance;
  }

  @Override
  public void periodic() {
    
    if (SBEnableChangePID.getDouble(0)==1){
      frontLeft.changeDriveKP(SBInputKp.getDouble(0));
      frontRight.changeDriveKP(SBInputKp.getDouble(0));
      rearLeft.changeDriveKP(SBInputKp.getDouble(0));
      rearRight.changeDriveKP(SBInputKp.getDouble(0));

    }
    swerveDriveOdometry.update(getDriveHeading(), getModulePositions());
    
    logData();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drive Speed",getTan());
    
    SmartDashboard.putNumber("Robot Heading", getDriveHeading().getDegrees());
    SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    
  }

  public void logData(){
     SmartDashboard.putNumber("left front angle", frontLeft.getAbsHeading().getDegrees());
     SmartDashboard.putNumber("left rear angle", rearLeft.getAbsHeading().getDegrees());
     SmartDashboard.putNumber("right front angle", frontRight.getAbsHeading().getDegrees());
    SmartDashboard.putNumber("right rear angle", rearRight.getAbsHeading().getDegrees());
  //   SmartDashboard.putNumber("left front angle", frontLeft.getAbsoluteCurrentAngle());
  //   SmartDashboard.putNumber("left rear angle", rearLeft.getAbsoluteCurrentAngle());
  //   SmartDashboard.putNumber("right front angle", frontRight.getAbsoluteCurrentAngle());
  //  SmartDashboard.putNumber("right rear angle", rearRight.getAbsoluteCurrentAngle());

   
    
    SmartDashboard.putNumber("left front velocity", frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("left rear velocity", rearLeft.getDriveVelocity());
    SmartDashboard.putNumber("right front velocity", frontRight.getDriveVelocity());
    SmartDashboard.putNumber("right rear velocity", rearRight.getDriveVelocity());


    SmartDashboard.putNumber("left front position", frontLeft.getDrivePosition());
    SmartDashboard.putNumber("left rear position", rearLeft.getDrivePosition());
    SmartDashboard.putNumber("right front position", frontRight.getDrivePosition());
    SmartDashboard.putNumber("right rear position", rearRight.getDrivePosition());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch().getValue());

   SmartDashboard.putNumber("left front angle", frontLeft.getAbsoluteCurrentAngle());
   SmartDashboard.putNumber("left front targetted angle", frontLeft.getCurrentTargetAngle());
   SmartDashboard.putNumber("left front targetted speed", frontLeft.getCurrentTargetSpeed());

  }

  public void setModuleStates(SwerveModuleState[] states){
        
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);


    SmartDashboard.putNumber("front left drive setpoint", states[0].speedMetersPerSecond);
}

public Rotation2d getDriveHeading(){
  return gyro.getRotation2d();
}

public ChassisSpeeds getSpeeds() {
  return kinematics.toChassisSpeeds(getModuleStates());
}

public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
  ChassisSpeeds targetSpeeds = robotRelativeSpeeds;
  SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,DriveConstants.MAX_TANGENTIAL_VELOCITY);

  setModuleStates(targetStates);
}

public double getDrivePitch(){
  return gyro.getPitch().getValue();
}

public void resetHeading(){
  gyro.reset();   
  }

  public void changeMax(){
    tanDeadband = 0.15;
    angDeadband = 0.15;
    driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
}

public void changeSlow(){
    tanDeadband = 0.20;
    angDeadband = 0.25;
    driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 3;
    driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 3;
}

public void changeVerySlow(){
    tanDeadband = 0.20;
    angDeadband = 0.25;
    driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 5;
    driveAngle =  DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 5;
}

public double getTan(){
  return driveSpeed;
}

public double getAng(){
  return driveAngle;
}

public double getTanDeadband(){
  return tanDeadband;
}

public double getAngDeadband(){
  return angDeadband;
}
public SwerveModulePosition[] getModulePositions(){
  SwerveModulePosition[] modulePositions = {frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};

  return modulePositions;
}

public SwerveModuleState[] getModuleStates(){
  SwerveModuleState[] moduleStates = {frontLeft.getModuleState(),frontRight.getModuleState(),rearLeft.getModuleState(),rearRight.getModuleState()};

  return moduleStates;
}

public ProfiledPIDController getRotationController(){
  return rotController;
}

public void lockModules(){
  frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
}

public void changeMotorGains(){
  // frontLeft.changeDriveKP();
  // frontRight.changeDriveKP();
  // rearLeft.changeDriveKP();
  // rearRight.changeDriveKP();

  frontLeft.changeTurnKP();
  frontRight.changeTurnKP();
  rearLeft.changeTurnKP();
  rearRight.changeTurnKP();
  
}

public Pose2d getPose2d(){
  return swerveDriveOdometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose2d){
  swerveDriveOdometry.resetPosition(getDriveHeading(), getModulePositions(), pose2d);
}



public Alliance getLastAlliance(){
  return lastKnownAlliance;
}

public void setLastAlliance(Alliance alliance){
  lastKnownAlliance = alliance;
}





private void stop(){
  frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
}



}
