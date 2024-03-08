// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//jay was here

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Arm.WaitForArmAngle;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Yeet extends SequentialCommandGroup {
  /** Creates a new Yeet. */
  public Yeet(Arm m_arm, Shooter m_shooter, Feeder m_feeder, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitForArmAngle(m_arm, ArmConstants.kYeetAngle), 
        new SetArmAngle(ArmConstants.kYeetAngle, m_arm),
        new IdleIntake(m_intake)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1), 
        new SetFeederSpeed(-1, m_feeder)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new SetShooterSpeed(m_shooter, ShooterConstants.kShooterSpeedYeetRPS)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5), 
        new SetFeederSpeed(10, m_feeder)
      ),
      new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder)
    );
  }
}
