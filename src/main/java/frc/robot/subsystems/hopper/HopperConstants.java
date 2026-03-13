package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class HopperConstants {

    // Command Constants
    public static final AngularVelocity feedingShootingVelocity = RotationsPerSecond.of(93);
    public static final AngularVelocity feedingEjectVelocity = RotationsPerSecond.of(-20);

    public static final AngularVelocity scramblerShootingVelocity = RotationsPerSecond.of(70);
    public static final AngularVelocity scramblerBeforeShootingVelocity = RotationsPerSecond.of(40);

    public static final AngularVelocity scramblerIdleVelocity = RotationsPerSecond.of(2);
    public static final AngularVelocity scramblerCollectVelocity = RotationsPerSecond.of(0);
    
    //Feeder Constants

    public static final int feederMotorCANID = 5;
    public static final double feederGearRatio = 1.0;

    public static final double feederKP = 0.8;
    public static final double feederKI = 0.0;
    public static final double feederKD = 0.0;
    public static final double feederKS = 0.0;
    public static final double feederKV = 0.132;
    public static final double feederKA = 0.0;

    public static final Current feederCurrentLimit = Amps.of(30.0) ;

    //Scrambler Constants
    public static final int scramblerMotorCANID = 6;
    public static final double scramblerGearRatio = 1.0;

    public static final double scramblerKP = 0.5;
    public static final double scramblerKI = 0.0;
    public static final double scramblerKD = 0.0;
    public static final double scramblerKS = 0.0;
    public static final double scramblerKV = 0.15;
    public static final double scramblerKA = 0.0;

    public static final Current scramblerCurrentLimit = Amps.of(30.0) ;
}