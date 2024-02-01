// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.Articulation.Arm;
import frc.robot.subsystems.Articulation.Shooter;


public class RevShooter extends Command {

private Shooter Shooter;
private double ShooterInput;
private DoubleSupplier spinSup;

private PIDController velController;
  public RevShooter(Shooter Shooter) {
    this.Shooter = Shooter;
    addRequirements(Shooter);

    velController = new PIDController(Constants.articulation.shooterP, Constants.articulation.shooterI, Constants.articulation.shooterD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 //Intake PID input
        Shooter.runShooter(velController.calculate(Shooter.getTopShooterRPM(), 2500));

SmartDashboard.putString("Shooter state", States.shooterState.toString());
SmartDashboard.putNumber("Shooter speed", ShooterInput);





  }

  public Boolean shooterSetpointReached() {
  return Shooter.getTopShooterRPM() > 2500;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return shooterSetpointReached();
  }
}
