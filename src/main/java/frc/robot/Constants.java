package frc.robot;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double armDeadband = 0.1;

    public static final class articulation {

        //articulator can IDs
        public static final int armLeft = 31;
        public static final int armRight = 32;

        public static final int intake = 33;
        
        public static final int shooter1 = 34; //bottom
        public static final int shooter2 = 35; //top

        //Intake sensor DIO port
        public static final int sensor = 0;

        //arm CANcoder ID
        public static final int armEncoder = 39;
        public static final double armEncoderOffset = 0;

        //Arm limits
        public static final int fwdLimit = 200;
        public static final int revLimit = 0;

        //Arm PID Constants
        public static final double armP = 0.04;
        public static final double armI = 0.0;
        public static final double armD = 0.0;
        public static final double armK = 0.0;

        //Shooter PID Constants
        public static final double shooterP = 0.0;
        public static final double shooterI = 0.0;
        public static final double shooterD = 0.0;
        public static final double shooterK = 0.0;

        //arm converstion math
        public static final double gearRatio = (5*5*5*(40/24));

        //Arm controller polling rate
        public static final double ScalingRatio = 2;
        

    }
}
