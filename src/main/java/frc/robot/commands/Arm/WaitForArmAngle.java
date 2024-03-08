// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class WaitForArmAngle extends Command {
  private final Arm m_arm;
  private final double m_angle;
  private final Timer m_timer;

  /** Creates a new WaitForArmAngle. */
  public WaitForArmAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_angle = angle;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngle(m_angle);
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get() < 0.3) {
      return false;
    } else {
      return m_arm.getController().atGoal();
    }
  }
}
