// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax steerMotor;
    private PIDController steerController;
    private CANcoder cancoder;

    private double encoderOffset;
    

    private double currentTargetAngle;
    private double currentTargetSpeed;

    private PIDController drivePIDController;

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> steerPosition;
    private StatusSignal<Double> steerVelocity;
    private BaseStatusSignal[] signals;
    private double driveRotationsPerMeter;

    private PositionVoltage angleSetter = new PositionVoltage(0);
    private VelocityVoltage velocitySetter = new VelocityVoltage(0);
     private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    private SwerveModulePosition internalState = new SwerveModulePosition();


    public SwerveModule(int drivePort, int turnPort, int encoderPort, double encoderOffset, boolean invertDrive){
        
        drivePIDController = new PIDController(0.00725, 0, 0);

        driveMotor = new TalonFX(drivePort);
        this.encoderOffset = encoderOffset;

        var talonConfigs = new TalonFXConfiguration();
        var driveConfigs = new Slot0Configs();

        driveConfigs.kP = 0.125;
      //  driveConfigs.kD = 0.0001;
        talonConfigs.Slot0  = driveConfigs;
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;       

        if(invertDrive)
            talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else 
            talonConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        for(int i=0;i<5;i++){
            initializationStatus =  driveMotor.getConfigurator().apply(talonConfigs);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + drivePort);

        }
        driveMotor.setInverted(invertDrive);
        

        steerMotor = new CANSparkMax(turnPort, MotorType.kBrushless);

        
        
        steerMotor.restoreFactoryDefaults();
        steerMotor.setSmartCurrentLimit(50);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setOpenLoopRampRate(0.2);
        steerMotor.setInverted(false);

        steerController = new PIDController(0, 0, 0);
        steerController.setPID(0.0075, 0, 0);
        steerController.enableContinuousInput(-180, 180);
        
        cancoder = new CANcoder(encoderPort);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        for(int i=0;i<5;i++){
            initializationStatus = cancoder.getConfigurator().apply(config);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + encoderPort);          
        }

        driveMotor.setPosition(0);
        drivePosition = driveMotor.getPosition();        
        driveVelocity = driveMotor.getVelocity();
        steerPosition = cancoder.getAbsolutePosition();
        steerVelocity = cancoder.getVelocity();
    
        signals = new BaseStatusSignal[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = steerPosition;
        signals[3] = steerVelocity;


    }

    public SwerveModulePosition getPosition(){
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();
        steerVelocity.refresh();
//*drivePosition.getTimestamp().getLatency()
        double drive_rot = drivePosition.getValue();
        double angle_rot = steerPosition.getValue()*steerPosition.getTimestamp().getLatency();

        internalState.distanceMeters = drive_rot*Constants.DriveConstants.metersPerWheelRotation;
        internalState.angle = Rotation2d.fromDegrees(((getAbsoluteCurrentAngle()+180)));


        return internalState;
    }


    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(driveVelocity.getValue()*Constants.DriveConstants.metersPerWheelRotation*(driveVelocity.getTimestamp().getLatency()), Rotation2d.fromDegrees(((getAbsoluteCurrentAngle()+180))));
    }

    public Double getAbsoluteCurrentAngle(){
        double angles =  cancoder.getAbsolutePosition().getValue()*360-encoderOffset;

        if (angles >180){
            angles = (angles-360);
        }

        else if (angles<=-180){
            angles = 360+ angles;
        }
        
        return angles;
   }

   
   public double closetAngle(double current, double target){
    double direction = Math.signum(target-current);
    double changes = Math.abs(target-current);
 
    if(Math.abs(current-target)>180){
        changes = Math.abs(360-changes);
        direction *=-1;
        
    }
    return changes*direction;

    
}

    public void setState(SwerveModuleState state){
        double velocityToSet = state.speedMetersPerSecond;
        if (Math.abs(velocityToSet) <= 0.01){
            driveMotor.set(0);
            steerMotor.set(0);
            return;
        }

        double angleToSetDeg = state.angle.getDegrees();

        double currentAngle = getAbsoluteCurrentAngle();

        
        if(Math.abs(closetAngle(currentAngle,angleToSetDeg))>90){
            
            if (angleToSetDeg >0){
                angleToSetDeg -=180;
            }
            else{
                angleToSetDeg+=180;  }

            velocityToSet *=-1;
        }
        
        steerMotor.set(-steerController.calculate( currentAngle,angleToSetDeg));
        currentTargetSpeed = velocityToSet;
        driveVelocity.refresh();

        

         driveMotor.set(drivePIDController.calculate(driveVelocity.getValue(),velocityToSet/(Constants.DriveConstants.metersPerWheelRotation)));
       // driveMotor.setControl(velocitySetter.withVelocity(velocityToSet/(Constants.DriveConstants.metersPerWheelRotation)));
    }

    public BaseStatusSignal[] getSignals(){
        return signals;
    }

    public double getDriveVelocity(){
        driveVelocity.refresh();
        return driveVelocity.getValue()*Constants.DriveConstants.metersPerWheelRotation;
       // return (driveVelocity.getValue()/1)*(Math.PI*1);
    }

    public double getDrivePosition(){
        drivePosition.refresh();
        return drivePosition.getValue()*Constants.DriveConstants.metersPerWheelRotation;
    }

    public void changeTurnKP(){
        steerController.setP(SmartDashboard.getNumber("Turn kP", 1));
    }

    public void changeDriveKP(double kPvalue){
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP =kPvalue;
        driveMotor.getConfigurator().refresh(driveConfig); //Override other settings?
    }

    public Rotation2d getAbsHeading(){
       // return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue());
         //Time 180 here?
         return Rotation2d.fromDegrees(getAbsoluteCurrentAngle()); //Time 180 here?

    }

    public Rotation2d getHeading(){
        return Rotation2d.fromRotations(steerMotor.getEncoder().getPosition());
    }

    public double getCurrentTargetSpeed(){
        return currentTargetSpeed;
    }

    public double getCurrentTargetAngle(){
        return currentTargetAngle;
    }



}
