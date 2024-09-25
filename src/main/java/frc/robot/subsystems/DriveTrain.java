// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

  public VictorSPX frontLeft;
  public VictorSPX frontRight;
  public VictorSPX backLeft;
  public VictorSPX backRight;
  public DriveTrain() {
    frontLeft = new VictorSPX(Constants.driveTrain.frontLeft);
    frontRight = new VictorSPX(Constants.driveTrain.frontRight);
    backLeft = new VictorSPX(Constants.driveTrain.backLeft);
    backRight = new VictorSPX(Constants.driveTrain.backRight);


    frontRight.setInverted(true);
    backRight.setInverted(true);

    gyro = new AHRS(SerialPort.Port.kMXP);

    
  }

  public void drive(double x, double y, double rx){
    double rotX = x * Math.cos(-getGyro()) - -y * Math.sin(-getGyro());
    double rotY = x * Math.sin(-getGyro()) + -y * Math.cos(-getGyro());

    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

    frontLeft.set(ControlMode.PercentOutput, frontLeftPower);
    frontRight.set(ControlMode.PercentOutput, frontRightPower);
    backLeft.set(ControlMode.PercentOutput, backLeftPower);
    backRight.set(ControlMode.PercentOutput, backRightPower);
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
