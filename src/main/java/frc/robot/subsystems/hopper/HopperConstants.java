package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class HopperConstants {

    // Command Constants
    public static final AngularVelocity feedingVelocity = RotationsPerSecond.of(20);
    public static final AngularVelocity scramblerActiveVelocity = RotationsPerSecond.of(10);
    
    //Feeder Constants

    public static final int feederMotorCANID = 5; //To be updated
    public static final double feederGearRatio = 1.0; //To be updated

    public static final double feederKP = 6.0; //To be updated
    public static final double feederKI = 0.0; //To be updated
    public static final double feederKD = 0.1; //To be updated
    public static final double feederKS = 0.0; //To be updated
    public static final double feederKV = 0.0; //To be updated
    public static final double feederKA = 0.0; //To be updated

    public static final AngularVelocity feederMaxVelocity = RotationsPerSecond.of(0.0); //To be updated

    public static final Current feederCurrentLimit = Amps.of(0.0); //To be updated

    //Scrambler Constants
    public static final int scramblerMotorCANID = 6; //To be updated
    public static final double scramblerGearRatio = 1.0; //To be updated

    public static final double scramblerKP = 5.0; //To be updated
    public static final double scramblerKI = 0.0; //To be updated
    public static final double scramblerKD = 0.0; //To be updated
    public static final double scramblerKS = 0.0; //To be updated
    public static final double scramblerKV = 0.0; //To be updated
    public static final double scramblerKA = 0.0; //To be updated

    public static final AngularVelocity scramblerMaxVelocity = RotationsPerSecond.of(0.0); //To be updated

    public static final Current scramblerCurrentLimit = Amps.of(0.0); //To be updated
}