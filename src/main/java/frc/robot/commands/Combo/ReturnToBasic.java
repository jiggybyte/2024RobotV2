// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ReturnToBasic extends ParallelCommandGroup {
  /** Creates a new ReturnToBasic. */
  public ReturnToBasic(Arm m_arm, Shooter m_shooter, Intake m_intake, Feeder m_feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmAngle(ArmConstants.kArmHomeAngle, m_arm),
      new SetFeederSpeed(0, m_feeder),
      new StopIntake(m_intake),
      new SetShooterSpeed(m_shooter, 0)
    );
  }
}
