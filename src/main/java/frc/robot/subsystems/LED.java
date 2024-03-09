// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private final Spark m_blinkin = new Spark(LEDConstants.kBlinkinPort);

  /** Creates a new LED. */
  public LED() {
    setAlliance();
  }

public void setLight(double setBlinkin) {
  m_blinkin.set(setBlinkin);
}

public void setAlliance() {
  if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    setLight(0.87);
  }else {
    setLight(0.61);
  }
}

public void noteLight() {

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
