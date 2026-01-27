package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public class ShooterConstants {

    public static final int shooter1CANID = 0;
    public static final int shooter2CANID = 1;
    public static final int shooter3CANID = 2;
    public static final int shooter4CANID = 3;

    public static final int hoodCANID = 4;


    public static final int currentLimit = 40; 
    public static final Time currentLimitTime = Seconds.of(1); 

    public static final double gearRatio = 1.0;


    public class PID {
            // shooter
            public static final double shooterkP = 0.0; 
            public static final double shooterkI = 0.0;
            public static final double shooterkD = 0.0;
            public static final double shooterkV = 0.0;
            public static final double shooterkA = 0.0;
            public static final double shooterkG = 0.0;
            public static final double shooterkS = 0.0;

            // hood
            public static final double hoodkP = 0.0; 
            public static final double hoodkI = 0.0;
            public static final double hoodkD = 0.0;
            public static final double hoodkV = 0.0;
            public static final double hoodkA = 0.0;
            public static final double hoodkG = 0.0;
            public static final double hoodkS = 0.0;
        }

        public class MotionMagic {

            // shooter
            public static final double shooterkMaxVelocity = 0.0;
            public static final double shooterkMaxAcceleration = 300.0;
            public static final double shooterkJerk = 0.0;

            // hood
            public static final double hoodkMaxVelocity = 0.0;
            public static final double hoodkMaxAcceleration = 300.0;
            public static final double hoodkJerk = 0.0;
        }

        public class SoftwareLimits {
            public static final double hoodMaxAngle = 0.0;
            public static final double hoodMinAngle = 0.0;
        }

}
