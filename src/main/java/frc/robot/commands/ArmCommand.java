// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.Articulation.Arm;

public class ArmCommand extends Command {
  private Arm Arm;
  private boolean setpointreached;
  private DoubleSupplier inputSup;

  private PIDController armController;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm Arm, DoubleSupplier inputSup) {
    this.Arm = Arm;
    this.inputSup = inputSup;
    addRequirements(Arm);

    armController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
    armController.setTolerance(0.25);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = MathUtil.applyDeadband(inputSup.getAsDouble(), Constants.armDeadband);

    double delta = input * Constants.articulation.ScalingRatio;
    
switch(States.armState){
            case standard:
                //standard
                 Arm.adjustSetpoint(delta);

                break;
            case low:

                //low
                  Arm.setAngle(0);
                break;
            case medium:

                //medium
                   Arm.setAngle(70);
                break;
            case high:

                //high
                  Arm.setAngle(100);
                break;
            case speakerShot:

                //high
                  Arm.setAngle(50);
                break;
          
}

     Arm.RunArm(armController.calculate(Arm.getAngle(), Arm.setpoint));
    SmartDashboard.putNumber("arm input", input);
    SmartDashboard.putNumber("arm angle", Arm.getAngle());
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
