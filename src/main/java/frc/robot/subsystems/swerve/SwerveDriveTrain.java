// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveAutoConstants.PathPlannerConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.GyroStuffs.PigeonV2;

public class SwerveDriveTrain extends SubsystemBase {
  /** Creates a new SwerveDriveTrain. */
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;



  private final PigeonV2 gyro;
  // private final SwerveDriveOdometry odometer;
  private boolean isTest = false;
  private final SwerveDrivePoseEstimator poseEstimator;
  //private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;
  private int counter = 0;
  private int visionFrequency = 1;


   private Field2d field;

   private double maxTangentialVelocity = 1.5;
   private double maxAngleVelocity = 2*Math.PI;



//    public enum SwerveModuleType {
//     MAG_ENCODER,
//     CANCODER
// }

// public enum DRIVE_MODE {
//     FIELD_ORIENTED,
//     ROBOT_ORIENTED,
//     AUTONOMOUS
// }


  /**
   * Construct a new {@link SwerveDrivetrain}
   */
  public SwerveDriveTrain(PigeonV2 gyro) {
    //Initializing the modules
                 frontLeft = new SwerveModule(
                    SwerveDriveConstants.kFLDriveID,
                    SwerveDriveConstants.kFLTurningID,
                    SwerveDriveConstants.kFLDriveReversed,
                    SwerveDriveConstants.kFLTurningReversed,
                    CANCoderConstants.kFLCANCoderID,
                    CANCoderConstants.kFLCANCoderReversed,
                    CANCoderConstants.kFLEncoderOffset);
                frontRight = new SwerveModule(
                    SwerveDriveConstants.kFRDriveID,
                    SwerveDriveConstants.kFRTurningID,
                    SwerveDriveConstants.kFRDriveReversed,
                    SwerveDriveConstants.kFRTurningReversed,
                    CANCoderConstants.kFRCANCoderID,
                    CANCoderConstants.kFRCANCoderReversed,
                    CANCoderConstants.kFREncoderOffset);
                backLeft = new SwerveModule(
                    SwerveDriveConstants.kBLDriveID,
                    SwerveDriveConstants.kBLTurningID,
                    SwerveDriveConstants.kBLDriveReversed,
                    SwerveDriveConstants.kBLTurningReversed,
                    CANCoderConstants.kBLCANCoderID,
                    CANCoderConstants.kBLCANCoderReversed,
                    CANCoderConstants.kBLEncoderOffset);
                backRight = new SwerveModule(
                    SwerveDriveConstants.kBRDriveID,
                    SwerveDriveConstants.kBRTurningID,
                    SwerveDriveConstants.kBRDriveReversed,
                    SwerveDriveConstants.kBRTurningReversed,
                    CANCoderConstants.kBRCANCoderID,
                    CANCoderConstants.kBRCANCoderReversed,
                    CANCoderConstants.kBREncoderOffset);

    this.gyro = gyro;

    this.poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());
            // this.odometer = new SwerveDriveOdometry(
        //     kDriveKinematics, 
        //     new Rotation2d(0), 
        //     getModulePositions()); 
        
    field = new Field2d();
    field.setRobotPose(poseEstimator.getEstimatedPosition());
     AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(
              PathPlannerConstants.kPPTranslationPIDConstants, 
              PathPlannerConstants.kPPRotationPIDConstants, 
              PathPlannerConstants.kPPMaxVelocity,
              SwerveDriveConstants.kTrackWidth
                ,new ReplanningConfig()), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);

  }



  @Override
  public void periodic() {
    runModules();
    poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    field.setRobotPose(poseEstimator.getEstimatedPosition());


    // This method will be called once per scheduler run
  }

     //****************************** RESETTERS ******************************/


    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
      // odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
      poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void refreshModulePID(double kPDrive, double kIDrive, double kDDrive,double kVDrive, double kPTurning, double kITurning, double kDTurning) {
      frontLeft.refreshPID(kPDrive,  kIDrive,  kDDrive, kVDrive,  kPTurning,  kITurning,  kDTurning);
      backLeft.refreshPID( kPDrive,  kIDrive,  kDDrive, kVDrive,  kPTurning,  kITurning,  kDTurning);
      frontRight.refreshPID( kPDrive,  kIDrive,  kDDrive, kVDrive,  kPTurning,  kITurning,  kDTurning);  
      backRight.refreshPID( kPDrive,  kIDrive,  kDDrive, kVDrive, kPTurning,  kITurning,  kDTurning);
  }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();
  }

      /**
     * Have modules move to their desired states. See {@link SwerveModule#run()} for more info.
     */
    public void runModules() {
      frontLeft.run();
      frontRight.run();
      backLeft.run();
      backRight.run();
  }


    //****************************** GETTERS ******************************/


    public double getTan(){
      return maxTangentialVelocity;
    }

    public double getAng(){
      return maxAngleVelocity;
    }


    public PigeonV2 getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Get the position of each swerve module
     * @return An array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

     private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
            )
        );
    }

    public void drive(double xSpeed, double ySpeed) {
        drive(xSpeed, ySpeed, 0);
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyro.getRotation2d())
            )
        );
    }
    public void driveFieldOriented(double xSpeed, double ySpeed) {
      driveFieldOriented(xSpeed, ySpeed, 0);
  }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
      SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      setModuleStates(targetStates);
  }

    public Rotation2d getDriveHeading(){
      return gyro.getRotation2d();
    }


      //****************************** SETTERS ******************************/

      public void setTan(double speed){
      maxTangentialVelocity=speed;
    }

    public void setAng(double speed){
      maxAngleVelocity = speed;
    }

      public void setVelocityControl(boolean withVelocityControl) {
        frontLeft.toggleVelocityControl(withVelocityControl);
        frontRight.toggleVelocityControl(withVelocityControl);
        backLeft.toggleVelocityControl(withVelocityControl);
        backRight.toggleVelocityControl(withVelocityControl);
    }

    /**
     * Set the neutral modes of all modules.
     * <p>
     * true sets break mode, false sets coast mode
     * 
     * @param breaking  Whether or not the modules should be in break
     */
    public void setBreak(boolean breaking, boolean turnbreaking ) {
        frontLeft.setBreak(breaking,turnbreaking);
        frontRight.setBreak(breaking,turnbreaking);
        backLeft.setBreak(breaking,turnbreaking);
        backRight.setBreak(breaking,turnbreaking);
    }

      /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      //  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void towModules() {
        frontLeft.setDesiredState(SwerveDriveConstants.towModuleStates[0], false);
        frontRight.setDesiredState(SwerveDriveConstants.towModuleStates[1], false);
        backLeft.setDesiredState(SwerveDriveConstants.towModuleStates[2], false);
        backRight.setDesiredState(SwerveDriveConstants.towModuleStates[3], false);
    }

     /**
     * Reset the odometry to the specified position.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
      // odometer.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
      poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

    public void initMainShuffleboard(int level) {

        //Im dumb, so 0 = off, 1 = everything, 2 = medium, 3 = minimal

        if (level == 0)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == 3) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Swerve");
        }

        switch (level) {
            case 0:
                break;
            case 1:

                tab.add("Field Position", field).withSize(6, 3);
                tab.addString(("Current Command"), () -> {
                    Command currCommand = this.getCurrentCommand();
                    if (currCommand == null) {
                        return "null";
                    } else {
                        return currCommand.getName();
                    }
                }
                );
                tab.add("Toggle Test", Commands.runOnce(() -> isTest = !isTest));
                tab.addBoolean("Test Mode", () -> isTest);
                // Might be negative because our swerveDriveKinematics is flipped across the Y axis
            case 2:
            case 3:
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addNumber("Odometry Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                break;
        }
    }

    public void initModuleShuffleboard(int level) {
      frontRight.initShuffleboard(level);
      frontLeft.initShuffleboard(level);
      backLeft.initShuffleboard(level);
      backRight.initShuffleboard(level);
  }














}