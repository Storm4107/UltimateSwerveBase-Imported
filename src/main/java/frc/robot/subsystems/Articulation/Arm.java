// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Articulation;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.rev.RevSwerveConfig;

public class Arm extends SubsystemBase {
  //arm PID controller
 private PIDController angleController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
  
 private CANSparkMax armLeft = new CANSparkMax(Constants.articulation.armLeft, MotorType.kBrushless);
 private CANSparkMax armRight = new CANSparkMax(Constants.articulation.armRight, MotorType.kBrushless);

 public CANCoderConfiguration CancoderConfig;

 public CANCoder armEncoder = new CANCoder(Constants.articulation.armEncoder);

 public double setpoint = Constants.articulation.armEncoderOffset;

  RelativeEncoder Encoder;
  /** Creates a new Arm. */
  public Arm() {

    //create arm motors
 
   armLeft.restoreFactoryDefaults();
   armRight.restoreFactoryDefaults();

   //Arm configuration
   armRight.setInverted(true);
   armLeft.setInverted(false);

   armRight.setSmartCurrentLimit(40);
   armLeft.setSmartCurrentLimit(40);

   armRight.setIdleMode(IdleMode.kBrake);
   armLeft.setIdleMode(IdleMode.kBrake);
   

   armLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
   armRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

   armLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
   armRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

   //Soft Limit sets
   armLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.articulation.fwdLimit);
   armRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.articulation.fwdLimit);

   armLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.articulation.revLimit);
   armRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.articulation.revLimit);
   
   armLeft.burnFlash();
   armRight.burnFlash();
  }

  //angle set
  public void setAngle(double angle){
    double  counts = Conversions.degreesToNeo(angle, Constants.articulation.gearRatio);
  }

  public void adjustSetpoint(double delta) {
   setpoint += delta;
  }

  //live adjustment
  public void RunArm(double input){
    armLeft.set(input);
    armRight.set(input);
  }

  //Encoder angle grab
   public double getAngle() {
        return armEncoder.getAbsolutePosition();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }
}
