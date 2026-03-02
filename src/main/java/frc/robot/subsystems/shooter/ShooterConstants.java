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

public class ShooterConstants {
    public static final int shooter1CANID = 0;
    public static final int shooter2CANID = 1;
    public static final int shooter3CANID = 2;

    public static final Current currentLimit = Amps.of(80);
    public static final Time currentLimitTime = Seconds.of(1);

    public static final double gearRatio = 1.04;

    public static final AngularVelocity shooterTolerance = RotationsPerSecond.of(1.0);

    public static final Distance allowedTrenchDistance = Meters.of(1.0);

    public static final Time hangTimeOnShot = Seconds.of(8/4.5);//number taken from video, 8 frames / 4.5 frames/sec seems to be about the right hang time. this should be tuned better later tho
    public static final Time dbRotationDelay = Seconds.of(0.5);
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
            public static final double shooterkP = 0.45; 
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

            // hood
            public static final double hoodkMaxVelocity = 0.0;
            public static final double hoodkMaxAcceleration = 300.0;
            public static final double hoodkJerk = 0.0;
        }

        public class SoftwareLimits {
            public static final double hoodMaxAngle = Positions.hoodHIGH;
            public static final double hoodMinAngle = 0.0;
        }

        public class Positions {
            // Field Positions
            public static final Translation2d blueHubPose = new Translation2d(4.5974,4.034536);
            public static final Translation2d redHubPose = new Translation2d(11.938,4.034536);

            public static final Distance allianceZone = Meters.of(4.5974);
            public static final Distance spinUpZone = Meters.of(6.0);

            public static final Distance blueAllianceWall = Meters.of(0);
            public static final Distance redAllianceWall = Meters.of(15);

            public static final Translation2d blueTargetLeft = new Translation2d(2,6);
            public static final Translation2d blueTargetRight = new Translation2d(2,2);

            public static final Translation2d redTargetLeft = new Translation2d(14,6);
            public static final Translation2d redTargetRight = new Translation2d(14,2);

            public static final double centerLineY = 4.034536;

            // Hood Setpoints
            public static final double hoodLOW = 0;
            public static final double hoodMEDIUM = 1;
            public static final double hoodHIGH = 2;
        
            public enum HubDistance {
                LOW(Meters.of(2)), // 0-2 m
                MEDIUM(Meters.of(3)), //  in
                HIGH(Meters.of(4)); // 96+ in

                private final Distance maxDistance;

                private HubDistance(Distance maxDistance) {
                    this.maxDistance = maxDistance;
                }

                public Distance maxDistance() { return maxDistance; }

                public static HubDistance fromDistance(Distance pos) {
                    for(HubDistance vol : values()) {
                        if (pos.baseUnitMagnitude() <= vol.maxDistance().baseUnitMagnitude()) {
                            return vol;
                        } 
                    }
                    return HubDistance.HIGH;
                }
            }
    }

    public class HoodPWMs {
            public static final int hoodLeftPWMPort = 2;
            public static final int hoodRightPWMPort = 0;
    }

}
