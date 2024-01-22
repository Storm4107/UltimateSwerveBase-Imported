// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Articulation;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax bottomShooter = new CANSparkMax(Constants.articulation.shooter1, MotorType.kBrushless);
  private CANSparkMax topShooter = new CANSparkMax(Constants.articulation.shooter2, MotorType.kBrushless);
  public Shooter() {

    bottomShooter.restoreFactoryDefaults();
    bottomShooter.setInverted(true);
    bottomShooter.setSmartCurrentLimit(40);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.burnFlash();

    topShooter.restoreFactoryDefaults();
    topShooter.setInverted(false);
    topShooter.setSmartCurrentLimit(40);
    topShooter.setIdleMode(IdleMode.kCoast);
    topShooter.burnFlash();

  }

   //shoot Command
   public void runShooter(double power) {
    topShooter.set(power);
    bottomShooter.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
