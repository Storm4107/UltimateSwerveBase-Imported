// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Articulation.Arm;
import frc.robot.subsystems.Articulation.Shooter;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     */
      return new InstantCommand(() -> States.shooterState = States.ShooterStates.shoot)
      .andThen(new InstantCommand(() -> States.armState = States.ArmStates.speakerShot))
      .andThen(new WaitCommand(3.0))
      .andThen(new InstantCommand(() -> States.intakeState = States.IntakeStates.shoot))
      .andThen(new WaitCommand(2.0))
      .andThen(new InstantCommand(() -> States.shooterState = States.ShooterStates.standard))
      .andThen(new InstantCommand(() -> States.armState = States.ArmStates.medium))
      .andThen(new InstantCommand(() -> States.intakeState = States.IntakeStates.standard));

       


    
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
