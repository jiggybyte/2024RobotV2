package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm.ArmDown;
import frc.robot.commands.Arm.ArmUp;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Climber.Arms.ForceArmsDown;
import frc.robot.commands.Climber.Arms.LowerArms;
import frc.robot.commands.Climber.Arms.RaiseArms;
import frc.robot.commands.Climber.Arms.ResetArms;
import frc.robot.commands.Climber.Arms.StopArms;
import frc.robot.commands.Climber.Hooks.LowerHooks;
import frc.robot.commands.Climber.Hooks.RaiseHooks;
import frc.robot.commands.Climber.Hooks.StopHooks;
import frc.robot.commands.Combo.AutoIntake;
import frc.robot.commands.Combo.AutoShoot;
import frc.robot.commands.Combo.FarShot;
import frc.robot.commands.Combo.MidShot;
import frc.robot.commands.Combo.ReturnToBasic;
import frc.robot.commands.Combo.ShortShot;
import frc.robot.commands.Combo.Manual.closeShot;
import frc.robot.commands.Combo.Manual.middleShot;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final PS5Controller m_driver = new PS5Controller(OIConstants.kDriverController);
    private final Joystick m_operator = new Joystick(OIConstants.kOperatorController);

    // Subsystems
    private final Swerve m_swerve = new Swerve();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();
    private final Shooter m_shooter = new Shooter();
    private final Limelight m_light = new Limelight();
    private final Climber m_climber = new Climber();

    //Shuffleboard & Auto
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final UsbCamera m_frontCamera;
    private final SendableChooser<Command> m_chooser;

    // NamedCommands@registercommand("AutonYaw", new AutonYaw(m_swerve));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -m_driver.getLeftY(), 
                () -> -m_driver.getLeftX(), 
                () -> -m_driver.getRightX(), 
                () -> m_driver.getCreateButton(),
                m_swerve
            )
        );

        // Commands that will show in PathPlanner
        NamedCommands.registerCommand("runIntake", new AutoIntake(m_feeder, m_intake, m_arm, m_shooter));
        NamedCommands.registerCommand("stopIntake", new StopIntake(m_intake));
        NamedCommands.registerCommand("closeShot", new closeShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("midShot", new middleShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("reset", new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        m_frontCamera = CameraServer.startAutomaticCapture(0);

        // Configure the button bindings
        configureButtonBindings();

        m_chooser = AutoBuilder.buildAutoChooser(); 

        m_tab.add("Auto Chooser", m_chooser);
        m_tab.add("Camera", m_frontCamera).withPosition(6, 0).withSize(4, 4).withWidget(BuiltInWidgets.kCameraStream);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
       // Driver Buttons
    //    new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new SimpleIntake(m_intake).alongWith(new SetFeederSpeed(10, m_feeder))).onFalse(new SetFeederSpeed(0, m_feeder));
       new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new AutoIntake(m_feeder, m_intake, m_arm, m_shooter)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
       new JoystickButton(m_driver, PS5Controller.Button.kCircle.value).whileTrue(new AutoShoot(m_shooter, m_swerve, m_light, m_feeder, m_arm, m_intake));
       new JoystickButton(m_driver, PS5Controller.Button.kOptions.value).onTrue(new ZeroHeading(m_swerve));
       new JoystickButton(m_driver, PS5Controller.Button.kR1.value).whileTrue(new SlowDrive(m_swerve));

       //Operator Buttons
        // Arm Commands
        new JoystickButton(m_operator, 1).whileTrue(new ArmUp(m_arm));
        new JoystickButton(m_operator, 6).whileTrue(new ArmDown(m_arm));
        new JoystickButton(m_operator, 4).onTrue(new SetArmAngle(0, m_arm));
        
        // Shooter Commands
        new JoystickButton(m_operator, 8).whileTrue(new ShortShot(m_arm, m_shooter, m_feeder, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_operator, 9).whileTrue(new MidShot(m_arm, m_shooter, m_intake, m_feeder)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_operator, 10).whileTrue(new FarShot(m_arm, m_shooter, m_feeder, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        // Climb Commands
        new JoystickButton(m_operator, 18).onTrue(new RaiseArms(m_climber)).onFalse(new StopArms(m_climber));
        new JoystickButton(m_operator, 20).onTrue(new LowerArms(m_climber));
        new JoystickButton(m_operator, 17).onTrue(new RaiseHooks(m_climber)).onFalse(new StopHooks(m_climber));
        new JoystickButton(m_operator, 19).onTrue(new LowerHooks(m_climber));
        new JoystickButton(m_operator, 22).onTrue(new ForceArmsDown(m_climber)).onFalse(new StopArms(m_climber));
        new JoystickButton(m_operator, 23).onTrue(new ResetArms(m_climber));

        // Amp Commands
        new JoystickButton(m_operator, 11).onTrue(new SetArmAngle(ArmConstants.kAmpAngle, m_arm)).onFalse(new SetArmAngle(0, m_arm));
        new JoystickButton(m_operator, 24).onTrue(new SetFeederSpeed(10, m_feeder).alongWith(new SetShooterSpeed(m_shooter, 10))).onFalse(new SetFeederSpeed(0, m_feeder).alongWith(new SetShooterSpeed(m_shooter, 0)));

        // Misc. Commands
        new JoystickButton(m_operator, 5).onTrue(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_operator, 21).onTrue(new SetShooterSpeed(m_shooter, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
