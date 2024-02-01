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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.swerve.rev.RevSwerveConfig;

public class Arm extends SubsystemBase {
  //arm PID controller
 private PIDController angleController = new PIDController(Constants.articulation.armP, Constants.articulation.armI, Constants.articulation.armD);
  
 private CANSparkMax armLeft = new CANSparkMax(Constants.articulation.armLeft, MotorType.kBrushless);
 private CANSparkMax armRight = new CANSparkMax(Constants.articulation.armRight, MotorType.kBrushless);

 public CANCoderConfiguration CancoderConfig;

 public CANCoder armEncoder = new CANCoder(Constants.articulation.armEncoder);

 public DigitalInput limitSwitch = new DigitalInput(Constants.articulation.limitSwitch);

 public double setpoint = 0;

  RelativeEncoder neoEncoder;
  
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

   neoEncoder = armLeft.getEncoder();
   neoEncoder.setPosition(0);
   
   
   armLeft.burnFlash();
   armRight.burnFlash();

  }

  //angle set
  public void setAngle(double angle){
    setpoint = angle;
  }

   public void setNeoZero(double position){
    neoEncoder.setPosition(position);
  }

  public void adjustSetpoint(double delta) {
    if (getNeoAngle() > Constants.articulation.fwdLimit) {
   if (delta > 0) {
    delta= 0;
    setpoint = Constants.articulation.fwdLimit;
   }

    } else{
      if (getNeoAngle() < Constants.articulation.revLimit) {
        if (delta < 0) {
    delta = 0;
    setpoint = 0;
   } else {
    delta = delta;
   }
      } 
       
    }
    setpoint += delta;
  }

  //live adjustment
  public void RunArm(double input){
    armLeft.set(input);
    armRight.set(input);
  }

  //Encoder angle grab
   public double getAngle() {
        return armEncoder.getAbsolutePosition() - Constants.articulation.armEncoderOffset;
    }

  
   public double getNeoAngle() {
    return neoEncoder.getPosition();
   }

   public double getNeoAdjustedAngle() {
    return (neoEncoder.getPosition() * 90/134.7);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("Neo raw position", getNeoAngle());
    SmartDashboard.putNumber("Neo position", getNeoAdjustedAngle());
    SmartDashboard.putString("Arm state", States.armState.toString());
    SmartDashboard.putBoolean("Arm bottom Limit", limitSwitch.get());
    SmartDashboard.putNumber("Arm CANCoder", getAngle());
  }
}
