package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.math.geometry.Translation2d;


public class ShooterConstants {

    public static final int shooter1CANID = 0;
    public static final int shooter2CANID = 1;
    public static final int shooter3CANID = 2;

    public static final int currentLimit = 40;
    public static final Time currentLimitTime = Seconds.of(1);

    public static final double gearRatio = 1.04 ;

    public static final AngularVelocity shooterTolerance = RotationsPerSecond.of(1.0);

    public static final Distance allowedTrenchDistance = Meters.of(1.0);

    public static final Angle hoodParkedAngle = Degrees.of(5.0) ;
    public static final Angle hoodMaxAngle = Degrees.of(75.0) ;
    public static final Angle hoodMinAngle = Degrees.of(0.0) ;

    public class PID {
        // shooter
        public static final double shooterkP = 0.0;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.0;
        public static final double shooterkV = 0.117;
        public static final double shooterkA = 0.0;
        public static final double shooterkG = 0.0;
        public static final double shooterkS = 0.0;
    }

    public class MotionMagic {

        // shooter
        public static final double shooterkMaxVelocity = 1000.0;
        public static final double shooterkMaxAcceleration = 3000.0;
        public static final double shooterkJerk = 0.0;
    }

    public class SoftwareLimits {
        public static final double hoodMaxAngle = 0.0;
        public static final double hoodMinAngle = 0.0;
    }

    public class HoodPWMs {
        public static final int hoodLeftPWMPort = 2;
        public static final int hoodRightPWMPort = 0;
    }

    public class ferryPositions{
        public static final Translation2d blueOutpostTarget= new Translation2d(2.135, 1.639);
        public static final Translation2d redOutpostTarget= new Translation2d(14.0,6.0);

    }
}
