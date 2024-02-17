// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.Articulation.Shooter;


public class ShooterCommand extends Command {

private Shooter Shooter;
private double ShooterInput;
private DoubleSupplier spinSup;
  public ShooterCommand(Shooter Shooter, DoubleSupplier spinSup) {
    this.Shooter = Shooter;
    this.spinSup = spinSup;
    addRequirements(Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 //Intake state switch
        switch(States.shooterState){
            case standard:

                //standard
                ShooterInput = ((spinSup.getAsDouble() + 1)/2);

                break;
            case shoot:

                //shoot
                  ShooterInput = 0.75;
                break;
            case spinup:
            ShooterInput = 0.2;
        }

//Intake input
Shooter.runShooter(ShooterInput);
SmartDashboard.putString("Shooter state", States.shooterState.toString());
SmartDashboard.putNumber("Shooter speed", ShooterInput);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
