// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Articulation;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax bottomShooter = new CANSparkMax(Constants.articulation.shooter1, MotorType.kBrushless);
  private CANSparkMax topShooter = new CANSparkMax(Constants.articulation.shooter2, MotorType.kBrushless);

  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  public Shooter() {

    bottomShooter.restoreFactoryDefaults();
    bottomShooter.setInverted(true);
    bottomShooter.setSmartCurrentLimit(5);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.burnFlash();

    topShooter.restoreFactoryDefaults();
    topShooter.setInverted(false);
    topShooter.setSmartCurrentLimit(40);
    topShooter.setIdleMode(IdleMode.kCoast);
    topShooter.burnFlash();
  

    topEncoder = topShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();

  }

   //shoot Command
   public void runShooter(double power) {
    topShooter.set(power);
    bottomShooter.set(power);
  }

  public double getTopShooterRPM() {
   return topEncoder.getVelocity();
  }

  public double getBottomShooterRPM() {
   return bottomEncoder.getVelocity();
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("top shooter", getTopShooterRPM());
    SmartDashboard.putNumber("bottom shooter", getBottomShooterRPM());
    // This method will be called once per scheduler run
  }
}
