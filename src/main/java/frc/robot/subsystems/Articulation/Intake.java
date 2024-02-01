// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Articulation;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
 private CANSparkMax Intake = new CANSparkMax(Constants.articulation.intake, MotorType.kBrushless);
 public DigitalInput sensor = new DigitalInput(Constants.articulation.sensor);

  public Intake() {
    //motor configuration
    Intake.restoreFactoryDefaults();
    Intake.setInverted(false);
    Intake.setSmartCurrentLimit(30);
    Intake.setIdleMode(IdleMode.kBrake);
    Intake.burnFlash();
  }
 
  //Intake Command
  public void runIntake(double power) {
    Intake.set(power);
  }

  public double getTemp() {
   return Intake.getMotorTemperature();
  }

  public double getSpeed() {
    return Intake.getAppliedOutput();
  }

   public boolean objectSensor() {
    return sensor.get();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake motor temperature", getTemp());
    SmartDashboard.putBoolean("object sensor", objectSensor());
  }
}
