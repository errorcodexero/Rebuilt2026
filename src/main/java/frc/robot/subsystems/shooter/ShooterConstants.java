package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;

public class ShooterConstants {
    public static final int shooter1CANID = 0;
    public static final int shooter2CANID = 1;
    public static final int shooter3CANID = 2;

    public static final Current currentLimit = Amps.of(50);
    public static final Time currentLimitTime = Seconds.of(0.2);

    //
    // The shooter velocity multiplier while the feeder is slow. This is to compensate for the 
    // fact that when the feeder is ramping, the shots are short
    //
    public static final double shooterVelocityMultiplierWhileFeederSlow = 1.1 ;

    public static final double gearRatio = 1.04;

    public static final AngularVelocity shooterTolerance = RotationsPerSecond.of(1.0);
    public static final AngularVelocity ejectVelocity = RotationsPerSecond.of(-20);

    public static final AngularVelocity minimumVelocitySetpoint = RotationsPerSecond.of(0.5);

    public static final Angle aimingTolerance = Degrees.of(5);
    public static final Angle defenseTolerance = Degrees.of(7.5);

    public static final Angle xWheelTolerance = Degrees.of(1);
    public static final Angle unXWheelTolerance = Degrees.of(3);

    public static final Distance allowedTrenchDistance = Meters.of(1.0);

    public static final double timeBeforeShoot = 0.2;

    public static final Angle hoodParkedAngle = Degrees.of(5.0) ;
    public static final Angle hoodMaxAngle = Degrees.of(75.0) ;
    public static final Angle hoodMinAngle = Degrees.of(0.0) ;

    public class FerryPositions{
        public static final Translation2d blueOutpostTarget= new Translation2d(2.135, 1.639);
        public static final Translation2d redOutpostTarget= new Translation2d(14.0,6.0);
    }

    public class PID {
            // shooter
            public static final double shooterkP = 0.5; 
            public static final double shooterkI = 0.0;
            public static final double shooterkD = 0.0;
            public static final double shooterkV = 0.132;
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

        public class Positions {
            // Field Positions
            public static final Translation2d blueHubPose = new Translation2d(4.5974,4.034536);
            public static final Translation2d redHubPose = new Translation2d(11.938,4.034536);

            public static final Distance allianceZone = Meters.of(4.5974);

            public static final Distance blueAllianceWall = Meters.of(0);
            public static final Distance redAllianceWall = Meters.of(FieldConstants.layout.getFieldLength());

            public static final Translation2d blueTargetLeft = new Translation2d(1,6);
            public static final Translation2d blueTargetRight = new Translation2d(1,2);

            public static final Translation2d redTargetLeft = new Translation2d(15,6);
            public static final Translation2d redTargetRight = new Translation2d(15,2);

            public static final Translation2d blueBumpTargetLeft = new Translation2d(4, FieldConstants.layout.getFieldWidth() - 2.5);
            public static final Translation2d blueBumpTargetRight = new Translation2d(4,2.5);

            public static final Translation2d redBumpTargetLeft = new Translation2d(FieldConstants.layout.getFieldLength() - 4, FieldConstants.layout.getFieldWidth() - 2.5);
            public static final Translation2d redBumpTargetRight = new Translation2d(FieldConstants.layout.getFieldLength() - 4,2.5);

            public static final double centerLineY = 4.034536;
    }

    public class HoodPWMs {
            public static final int hoodLeftPWMPort = 2;
            public static final int hoodRightPWMPort = 0;
    }

    public class ferryPositions{
        public static final Translation2d blueOutpostFerryTarget= new Translation2d(2.136,1.935); //ferry target for the blue outpost
        public static final Translation2d redOutpostFerryTarget= new Translation2d(14.0,6.0); //ferry target for the red outpost
        public static final Translation2d blueDepotFerryTarget= new Translation2d(2.0,6.0); //ferry target for the blue depot
        public static final Translation2d redDepotFerryTarget= new Translation2d(14.0,2.0); //ferry target for the red depot
    }
}
