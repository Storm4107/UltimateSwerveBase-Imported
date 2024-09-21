// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends Command {
  private DriveTrain m_Drive;
  private DoubleSupplier xsup;
  private DoubleSupplier ysup;
  private DoubleSupplier zsup;
  private DoubleSupplier speedDial;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrain m_Drive, DoubleSupplier xsup, DoubleSupplier ysup, DoubleSupplier zsup, DoubleSupplier speedDial) {
  this.m_Drive = m_Drive;
  addRequirements(m_Drive);

  this.xsup = xsup;
  this.ysup = ysup;
  this.zsup = zsup;
  this.speedDial = speedDial;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xval = MathUtil.applyDeadband(xsup.getAsDouble(), Constants.stickDeadband) * ((speedDial.getAsDouble() + 1) / 2);
    double yval = MathUtil.applyDeadband(ysup.getAsDouble(), Constants.stickDeadband) * ((speedDial.getAsDouble() + 1) / 2);
    double zval = MathUtil.applyDeadband(zsup.getAsDouble(), Constants.stickDeadband) * ((speedDial.getAsDouble() + 1) / 2);

    m_Drive.drive(xval, yval, zval);
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
