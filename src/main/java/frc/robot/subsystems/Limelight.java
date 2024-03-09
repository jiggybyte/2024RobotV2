// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {
  private GenericEntry m_distance;
  private ShuffleboardTab m_tab;

  /** Creates a new Limelight. */
  public Limelight() {
    setCorrectTarget();
    m_tab = Shuffleboard.getTab("Main");
    m_distance = m_tab.add("Distance", getDistance()).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  public double getDistance() {
    return (LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("")));
  }

  public double distanceToArmAngle(double distance) {
    return (32 + (distance * 5.98) + (10.7 * distance * distance) - (6.66 * distance * distance * distance) + (1.07 * distance * distance * distance * distance));
  }

  public double getTX() {
    return LimelightHelpers.getTX("");
  }

  public double getTY() {
    return LimelightHelpers.getTY("");
  }

  public boolean hasCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue && LimelightHelpers.getFiducialID("") == 1) {
      return true;
    } else if (DriverStation.getAlliance().get() == Alliance.Red && LimelightHelpers.getFiducialID("") == 3) {
      return true;
    } else {
      return false;
    }
  }

  private void setCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPriorityTagID("", 1);
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("", 3);
    } else {
      DriverStation.reportError("Did not get alliance to setup Limelight.", true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_distance.setDouble(getDistance());
  }
}
