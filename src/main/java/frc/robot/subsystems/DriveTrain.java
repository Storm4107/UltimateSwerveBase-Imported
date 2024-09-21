// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private MecanumDrive Drive;
  public DriveTrain() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.driveTrain.frontLeft);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(Constants.driveTrain.frontRight);
    WPI_TalonSRX backLeft = new WPI_TalonSRX(Constants.driveTrain.backLeft);
    WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.driveTrain.backRight);

    Drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }

  public void drive(double x, double y, double z){
    Drive.driveCartesian(z, y, x, getGyro());
  }

  public Rotation2d getGyro(){
    return new Rotation2d();
  }

  public void zeroGyro(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
