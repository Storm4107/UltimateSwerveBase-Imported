// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Articulation.Intake;


public class IntakeCommand extends Command {

private Intake Intake;
private double IntakeInput;
  public IntakeCommand(Intake Intake) {
    this.Intake = Intake;
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 //Intake state switch
        switch(States.intakeState){
            case standard:

                //standard
                  IntakeInput = 0;

                break;
            case eject:

                //eject
                  IntakeInput = -0.8;
                break;
            case shoot:

                //shoot
                  IntakeInput = 0.8;
                break;


            case hold:
            
                //hold
                  IntakeInput = 0;
                break;

            case intake:

                //intake
                if (Intake.objectSensor()) {
                  IntakeInput = 1;
                }
                else {
                  IntakeInput = 0;
                  }
               //TODO: Add sensor input
        }

//Intake input
Intake.runIntake(IntakeInput);
SmartDashboard.putString("intake state", States.intakeState.toString());
SmartDashboard.putNumber("Intake speed", IntakeInput);


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
