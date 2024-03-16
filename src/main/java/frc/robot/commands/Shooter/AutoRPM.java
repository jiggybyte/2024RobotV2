// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoRPM extends Command {
  private final Shooter m_shooter;
  private final Limelight m_light;

  /** Creates a new AutoRPM. */
  public AutoRPM(Shooter shooter, Limelight light) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_light = light;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setSpeed(m_light.getTargetRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
