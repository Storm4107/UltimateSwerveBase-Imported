package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Articulation.Arm;
import frc.robot.subsystems.Articulation.Intake;
import frc.robot.subsystems.Articulation.Shooter;
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
    private final RevSwerve s_Swerve = new RevSwerve();
    private final Arm s_arm = new Arm();
    private final Intake s_intake = new Intake();
    private final Shooter s_shooter = new Shooter();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> -driver.getRawAxis(speedDial) 
            )
        );

       s_arm.setDefaultCommand(
        new ArmCommand(
            s_arm,
            () -> -operator.getRawAxis(armAxis)
             )
       );

       s_intake.setDefaultCommand(
        new IntakeCommand(
            s_intake
            )
       );

       s_shooter.setDefaultCommand(
        new ShooterCommand(
            s_shooter,
            () -> operator.getRawAxis(shooterSpeedDial)
        )
       );



        // Configure the button bindings
        configureButtonBindings();
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

        

       
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
