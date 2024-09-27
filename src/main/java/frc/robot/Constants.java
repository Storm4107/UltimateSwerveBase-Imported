package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double armDeadband = 0.3;

    public static final class articulation {

        //articulator can IDs
        public static final int armLeft = 31;
        public static final int armRight = 32;

        public static final int intake = 33;
        
        public static final int shooter1 = 34; //bottom
        public static final int shooter2 = 35; //top

        //Intake sensor DIO port
        public static final int sensor = 9;
        public static final int limitSwitch = 8;

        //Arm limits
        public static final int fwdLimit = 2000;
        public static final int revLimit = -3;

        //Arm PID Constants
        public static final double armP = 0.04;
        public static final double armI = 0.00;
        public static final double armD = 0.00;
        public static final double armK = 0.0;

        //arm FeedForward Constants
        public static final double armFF = 0.00;

        //arm converstion math
        public static final double gearRatio = (5*5*5);
        //60/14 sprocket ratio
        public static final double ChainRatio = 8.91;

        //Arm controller polling rate
        public static final double ScalingRatio = 1.5;
        
    }

    public static final class driveTrain {
        //TODO: set drive Motor IDS
        public static final int frontLeft = 1;
        public static final int frontRight = 0;
        public static final int backLeft = 9;
        public static final int backRight = 8;
    }
}
