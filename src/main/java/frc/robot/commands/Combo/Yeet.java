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
  public Yeet(Arm arm, Shooter shooter, Feeder feeder, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitForArmAngle(() -> ArmConstants.kYeetAngle, arm), 
        new SetArmAngle(ArmConstants.kYeetAngle, arm),
        new IdleIntake(intake)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1), 
        new SetFeederSpeed(-1, feeder)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.75), 
        new SetShooterSpeed(ShooterConstants.kShooterSpeedYeetRPS, shooter)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5), 
        new SetFeederSpeed(10, feeder)
      ),
      new ReturnToBasic(arm, shooter, intake, feeder)
    );
  }
}
