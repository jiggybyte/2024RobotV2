// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.Hooks;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RaiseHooks extends Command {
  private final Climber m_climber;
  /** Creates a new RaiseHooks. */
  public RaiseHooks(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.lowerHooks();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopHooks();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climber.getHookEncoder() >= 5) {
      return true;
    } else {
      return false;
    }
  }
}
