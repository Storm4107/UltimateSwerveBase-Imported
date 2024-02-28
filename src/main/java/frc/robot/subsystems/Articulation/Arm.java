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
  
 private CANSparkMax armLeft = new CANSparkMax(Constants.articulation.armLeft, MotorType.kBrushless);
 private CANSparkMax armRight = new CANSparkMax(Constants.articulation.armRight, MotorType.kBrushless);

 public DigitalInput limitSwitch = new DigitalInput(Constants.articulation.limitSwitch);

 public double setpoint = 0;
 
 public double armRatio = Constants.articulation.ChainRatio * Constants.articulation.gearRatio;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  
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

   leftEncoder = armLeft.getEncoder();
   leftEncoder.setPositionConversionFactor(.7);
   leftEncoder.setPosition(0);

   rightEncoder = armRight.getEncoder();
   rightEncoder.setPositionConversionFactor(.7);
   rightEncoder.setPosition(0);

   armLeft.burnFlash();
   armRight.burnFlash();
  }

  //angle set
  public void setAngle(double angle){
    setpoint = angle;
  }

   public void setNeoZero(double position){
    leftEncoder.setPosition(position);
    rightEncoder.setPosition(position);
    setpoint = position;
  }

  public void adjustSetpoint(double delta) {
    if (getLeftAngle() > Constants.articulation.fwdLimit) {
   if (delta > 0) {
    delta= 0;
    setpoint = Constants.articulation.fwdLimit;
   }

    } else{
      if (getLeftAngle() < Constants.articulation.revLimit) {
        if (delta < 0) {
    delta = 0;
    setpoint = Constants.articulation.revLimit;
   } else {
    delta = delta;
   }
      } 
       
    }
    setpoint += delta;
  }

  //live adjustment
  public void RunLeftArm(double input){
    armLeft.set(input + leftFF());
  }

  public void RunRightArm(double input){
    armRight.set(input + rightFF());
  }

  public double leftFF(){
    double FFOutput = Math.cos(getLeftAngle() * (Math.PI/180) * Constants.articulation.armFF);
   // return FFOutput;
   return 0;
  }

  public double rightFF(){
    double FFOutput = Math.cos(getLeftAngle() * (Math.PI/180) * Constants.articulation.armFF);
   // return FFOutput;
   return 0;
  }

  //Encoder angle grab 
  public double getLeftAngle() {
    return (leftEncoder.getPosition() - 2.5);
   }

  public double getRightAngle() {
    return (rightEncoder.getPosition() - 2.5);
   }

  public double getArmAngle() {
    return (getLeftAngle() + getRightAngle() / 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("Neo position", getArmAngle());
    SmartDashboard.putNumber("Neo Conversion factor", leftEncoder.getPositionConversionFactor());
    SmartDashboard.putString("Arm state", States.armState.toString());
    SmartDashboard.putNumber("Left input", armLeft.getAppliedOutput());
    SmartDashboard.putNumber("Right input", armRight.getAppliedOutput());
  }
}
