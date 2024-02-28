// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.Articulation.Arm;

public class ArmCommand extends Command {

  private Arm Arm;
  private DoubleSupplier inputSup;
  private PIDController leftController;
  private PIDController rightController;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm Arm, DoubleSupplier inputSup) {
    this.Arm = Arm;
    this.inputSup = inputSup;
    addRequirements(Arm);

    leftController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
    leftController.setTolerance(0.001);

    rightController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
    rightController.setTolerance(0.001);

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

                //low ground
                  Arm.setAngle(-2.5);
                break;
            case medium:

                //medium source transit
                   Arm.setAngle(60);
                break;
            case high:

                //high amp
                  Arm.setAngle(90);
                break;
            case speakerShot:

                //Speaker shoot
                  Arm.setAngle(21.5);
                break;
            case podiumShot:

                //podium shoot
                  Arm.setAngle(45);
                break;
            
          
}

     Arm.RunLeftArm(leftController.calculate(Arm.getLeftAngle(), Arm.setpoint));
     Arm.RunRightArm(rightController.calculate(Arm.getRightAngle(), Arm.setpoint));
    SmartDashboard.putNumber("left Angle", Arm.getLeftAngle());
    SmartDashboard.putNumber("right Angle", Arm.getRightAngle());
    SmartDashboard.putNumber("Setpoint", Arm.setpoint);
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
