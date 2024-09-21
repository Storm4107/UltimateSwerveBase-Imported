// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private MecanumDrive Drive;
  public AHRS gyro;
  public DriveTrain() {
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.driveTrain.frontLeft);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(Constants.driveTrain.frontRight);
    WPI_TalonSRX backLeft = new WPI_TalonSRX(Constants.driveTrain.backLeft);
    WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.driveTrain.backRight);

    gyro = new AHRS(SerialPort.Port.kMXP);

    Drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }

  public void drive(double x, double y, double z){
    Drive.driveCartesian(z, y, x, new Rotation2d(gyro.getYaw()));
  }

  public double getGyro(){
    return Math.toRadians(gyro.getYaw());
  }

  public void zeroGyro(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
