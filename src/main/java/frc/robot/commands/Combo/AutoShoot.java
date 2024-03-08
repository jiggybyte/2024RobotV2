// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Arm.WaitForArmAngle;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.commands.Shooter.WaitForShooterSpeed;
import frc.robot.commands.Swerve.PIDTurning;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Swerve swerve, Limelight light, Feeder feeder, Arm arm, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new WaitForShooterSpeed(shooter, shooter.distanceToRPM(light.getDistance())),
        new PIDTurning(swerve, light),
        new WaitForArmAngle(arm, arm.distanceToArmAngle(light.getDistance()))
      ),
      new SetFeederSpeed(10, feeder),
      new WaitCommand(0.25),
      new ParallelCommandGroup(
        new SetShooterSpeed(shooter, 0),
        new SetFeederSpeed(0, feeder),
        new SetArmAngle(0, arm)
      )
    );
  }
}
