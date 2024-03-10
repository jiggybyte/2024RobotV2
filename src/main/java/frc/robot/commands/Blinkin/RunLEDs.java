// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Blinkin;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LED;

public class RunLEDs extends Command {
  private final LED m_led;
  private final Feeder m_feeder;

  /** Creates a new RunLEDs. */
  public RunLEDs(LED led, Feeder feeder) {
    m_led = led;
    m_feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_feeder.haveNote()) {
      m_led.setLight(0.35);
    } else {
      m_led.setAlliance();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
