package frc.robot;

public class States {
//Drive states
    public static enum DriveStates {
        standard, d0, d90, d180, d270
    }
    public static DriveStates driveState = DriveStates.standard;

//intake states
  public static enum IntakeStates {
        standard, intake, eject, hold, shoot
    }
    public static IntakeStates intakeState = IntakeStates.standard;
    
//shooter states
  public static enum ShooterStates {
        standard, spinup, shoot
    }
    public static ShooterStates shooterState = ShooterStates.spinup;
}
