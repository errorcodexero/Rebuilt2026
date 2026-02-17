package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ShooterConstants {

    public static final int shooter1CANID = 0;
    public static final int shooter2CANID = 1;
    public static final int shooter3CANID = 2;

    public static final int currentLimit = 40;
    public static final Time currentLimitTime = Seconds.of(1);

    public static final double gearRatio = 1.04 ;

    public static final AngularVelocity shooterTolerance = RotationsPerSecond.of(1.0);

    public static final Distance allowedTrenchDistance = Meters.of(1.0);

    public static final Time hangTimeOnShot = Seconds.of(7/4.5);

    public class PID {
            // shooter
            public static final double shooterkP = 0.5; 
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
            public static final double shooterkMaxVelocity = 1000.0;
            public static final double shooterkMaxAcceleration = 3000.0;
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

        public class Positions {
            public static final Translation2d blueHubPose = new Translation2d(4.5974,4.034536);
            public static final Translation2d redHubPose = new Translation2d(11.938,4.034536);

            // Hood Setpoints
            public static final double hoodLOW = 0;
            public static final double hoodMEDIUM = 1;
            public static final double hoodHIGH = 2;

            // Tested Distances 
            public static final double dist1 = 1;
            public static final double dist2 = 2;
            public static final double dist3 = 3;

            public static final double dist4 = 4;
            public static final double dist5 = 5;
            public static final double dist6 = 6;

            public static final double dist7 = 7;
            public static final double dist8 = 8;
            public static final double dist9 = 9;

            // Tested Velocities
            public static final double low1 = 10;
            public static final double low2 = 20;
            public static final double low3 = 30;

            public static final double med1 = 40;
            public static final double med2 = 50;
            public static final double med3 = 60;

            public static final double high1 = 70;
            public static final double high2 = 80;
            public static final double high3 = 90;

            public static final InterpolatingDoubleTreeMap distMap = new InterpolatingDoubleTreeMap();

            public static void initMap() {
                distMap.put(dist1, low1);
                distMap.put(dist2, low2);
                distMap.put(dist3, low3);
                distMap.put(dist4, med1);
                distMap.put(dist5, med2);
                distMap.put(dist6, med3);
                distMap.put(dist7, high1);
                distMap.put(dist8, high2);
                distMap.put(dist9, high3);
            }
        
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
