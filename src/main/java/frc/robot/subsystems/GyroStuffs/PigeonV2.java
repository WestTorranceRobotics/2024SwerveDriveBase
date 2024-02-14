// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GyroStuffs;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonV2 extends SubsystemBase {
  /** Creates a new PigeonV2. */
  private Pigeon2 pigeon;
  private double offset, pitchOffset, rollOffset = 0;

  public PigeonV2(int id) {
            try {
            this.pigeon = new Pigeon2(id);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Pigeon 2 over CAN: " + ex.getMessage(), true);
        }

        offset = 0;
        pitchOffset = 0;
        rollOffset = 0;
  }

  public void zeroAll() {
    zeroHeading();
    zeroPitch();
    zeroRoll();
}
  public void zeroHeading() {
    pigeon.setYaw(0);
    offset = 0;
  }


    /**
     * Return the internal pigeon object.
     * @return
     */
    public Pigeon2 getPigeon() {
      return this.pigeon;
  }

    public void zeroPitch() {
        this.pitchOffset = -pigeon.getPitch().getValue();
    }
    
    public void zeroRoll() {
        this.rollOffset = pigeon.getRoll().getValue();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setPitchOffset(double offset) {
        this.pitchOffset = offset;
    }

    public void setRollOffset(double offset) {
        this.rollOffset = offset;
    }

    public void resetHeading(double headingDegrees) {
        zeroHeading();
        offset = headingDegrees;
    }

    public void resetPitch(double pitchDegrees) {
      this.pitchOffset = this.getPitch() - pitchDegrees;
  }

  public void resetRoll(double rollDegrees) {
      this.rollOffset = this.getRoll() - rollDegrees;
  }

  public double getHeading() {
      return -pigeon.getAngle() - offset;
  }

  public double getYaw() {
      double currentYaw = (pigeon.getYaw().getValue() - offset) % 360;
      if (currentYaw < 0) {
          return currentYaw + 360;
      } else {
          return currentYaw;
      }
  }

  public double getPitch() {
      return (-pigeon.getPitch().getValue() - pitchOffset) % 360;
  }

  public double getRoll() {
      return (pigeon.getRoll().getValue() - rollOffset) % 360;
  }

  public double getHeadingOffset() {
      return this.offset;
  }


 public double getRollOffset() {
        return this.rollOffset;
    }

    public double getPitchOffset() {
        return this.pitchOffset;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    /**
     * For orientations, see page 20 of {@link https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User%27s%20Guide.pdf}
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            Math.toRadians(getRoll()),
            Math.toRadians(getPitch()),
            Math.toRadians(getHeading())
        );
    }
    public void reportToSmartDashboard() {
        
    }
     public void initShuffleboard() {

    } 

}