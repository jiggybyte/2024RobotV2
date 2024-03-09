package frc.robot;

import com.pathplanner.lib.auto.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Blinkin.RunLEDs;
import frc.robot.commands.Climber.Arms.*;
import frc.robot.commands.Climber.Hooks.*;
import frc.robot.commands.Combo.*;
import frc.robot.commands.Combo.Manual.*;
import frc.robot.commands.Feeder.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
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
    private final LED m_led = new LED();

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

        m_led.setDefaultCommand(new RunLEDs(m_led, m_feeder));

        // Commands that will show in PathPlanner
        NamedCommands.registerCommand("runIntake", new AutoIntake(m_feeder, m_intake, m_arm, m_shooter));
        NamedCommands.registerCommand("stopIntake", new StopIntake(m_intake));
        NamedCommands.registerCommand("closeShot", new closeShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("midShot", new middleShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("reset", new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("ampDrop", new AmpDrop(m_arm, m_intake, m_shooter, m_feeder));

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
        //new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new SimpleIntake(m_intake).alongWith(new SetFeederSpeed(10, m_feeder))).onFalse(new SetFeederSpeed(0, m_feeder));
        new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new AutoIntake(m_feeder, m_intake, m_arm, m_shooter)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_driver, PS5Controller.Button.kCircle.value).whileTrue(new AutoShoot(m_shooter, m_swerve, m_light, m_feeder, m_arm, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
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

        new JoystickButton(m_operator, 16).onTrue(new SetArmAngle(54, m_arm).alongWith(new SetShooterSpeed(m_shooter, 56))).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        new JoystickButton(m_operator, 7).onTrue(new SetFeederSpeed(10, m_feeder)).onFalse(new SetFeederSpeed(0, m_feeder));
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
