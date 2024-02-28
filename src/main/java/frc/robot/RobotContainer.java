package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Articulation.Arm;
import frc.robot.subsystems.Articulation.Intake;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Articulation.Shooter;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.swerve.rev.RevSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;
    private final int speedDial = 5;

    //operator controls
    private final int armAxis = 1;
    private final int shooterSpeedDial = 3;

    //operator presets
    private JoystickButton high = new JoystickButton(operator, 8);
    private JoystickButton medium = new JoystickButton(operator, 10);
    private JoystickButton low = new JoystickButton(operator, 12);
    private JoystickButton armControl = new JoystickButton(operator, 11);
    private JoystickButton speakerShot = new JoystickButton(operator, 5);
    private JoystickButton revShot = new JoystickButton(operator, 3);
    private JoystickButton shooterOff = new JoystickButton(operator, 4);
    private JoystickButton podiumShot = new JoystickButton(operator, 6);

    //intake buttons
    private JoystickButton shoot = new JoystickButton(operator, 1); //trigger
    private JoystickButton intake = new JoystickButton(operator, 7);
    private JoystickButton eject = new JoystickButton(operator, 9);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 7);

    private final JoystickButton dampen = new JoystickButton(driver, 18);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final RevSwerve s_Swerve = new RevSwerve(s_PoseEstimator);
    private final Vision s_Vision = new Vision(s_PoseEstimator);
    private final Arm s_arm = new Arm();
    private final Intake s_intake = new Intake();
    private final Shooter s_shooter = new Shooter();
  

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Swerve Command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> -driver.getRawAxis(speedDial) 
            )
        );

       //Arm command
       s_arm.setDefaultCommand(
        new ArmCommand(
            s_arm,
            () -> -operator.getRawAxis(armAxis)
             )
       );

       //Intake Command
       s_intake.setDefaultCommand(
        new IntakeCommand(
            s_intake
            )
       );

       //shooter command
       s_shooter.setDefaultCommand(
        new ShooterCommand(
            s_shooter,
            () -> operator.getRawAxis(shooterSpeedDial)
        )
       );

        // Configure the button bindings
        configureButtonBindings();

        //Pathplanner commands - templates
        NamedCommands.registerCommand("Shooter Position", new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.shoot),
                new InstantCommand(() -> States.armState = States.ArmStates.speakerShot)
            ));

        NamedCommands.registerCommand("Pickup Position", new ParallelCommandGroup(
                new InstantCommand(() -> States.intakeState = States.IntakeStates.intake),
                new InstantCommand(() -> States.armState = States.ArmStates.low)
            ));

        NamedCommands.registerCommand("EjectOn", new InstantCommand(() -> States.intakeState = States.IntakeStates.shoot));

        NamedCommands.registerCommand("EjectOff", new InstantCommand(() -> States.intakeState = States.IntakeStates.standard));

        NamedCommands.registerCommand("Position Reset",  new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.standard),
                new InstantCommand(() -> States.armState = States.ArmStates.medium),
                new InstantCommand(() -> States.intakeState = States.IntakeStates.standard)
            ));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );

        //Intake states
        shoot.onTrue(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.shoot)).onFalse(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.standard)
        );
        eject.onTrue(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.eject)).onFalse(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.standard)
        );
        intake.onTrue(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.intake)).onFalse(
            new InstantCommand(() -> States.intakeState = States.IntakeStates.standard)
        );

        //Arm presets
        low.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.standard),
                new InstantCommand(() -> States.armState = States.ArmStates.low)
            )
        );
        medium.onTrue(
             new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.standard),
                new InstantCommand(() -> States.armState = States.ArmStates.medium)
            )
        );
        high.onTrue(
             new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.spinup),
                new InstantCommand(() -> States.armState = States.ArmStates.high)
            )
        );
        speakerShot.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.shoot),
                new InstantCommand(() -> States.armState = States.ArmStates.speakerShot)
            )
            
        );
        podiumShot.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> States.shooterState = States.ShooterStates.shoot),
                new InstantCommand(() -> States.armState = States.ArmStates.podiumShot)
            )
            
        );
        armControl.onTrue(
            new InstantCommand(() -> States.armState = States.ArmStates.standard)
        );
        revShot.onTrue(
            new InstantCommand(() -> States.shooterState = States.ShooterStates.spinup)
        );
        shooterOff.onTrue(
            new InstantCommand(() -> States.shooterState = States.ShooterStates.standard)
        );
          
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
