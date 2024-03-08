// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmUp extends Command {
  private final Arm m_arm;
  private boolean m_pastLimit = false;

  /** Creates a new ArmUp. */
  public ArmUp(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_pastLimit) {
      m_arm.setAngle(ArmConstants.kMaxAngle);
    } else {
      m_arm.setAngle(m_arm.getAngle());
    }
    m_arm.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.getAngle() > ArmConstants.kMaxAngle) {
      m_pastLimit = true;
      return true;
    } else {
      return false;
    }
  }
}
