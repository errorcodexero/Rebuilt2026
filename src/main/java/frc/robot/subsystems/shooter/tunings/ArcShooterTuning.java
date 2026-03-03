package frc.robot.subsystems.shooter.tunings;

public class ArcShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final double touchingDist[] = { 1.616 , 1.86, 2.18} ;
    private static final double touchingVelocities[] = {  55.0, 58.0, 65.0 } ;

    private static final double lowHood = 15 ;
    private static final double lowDist[] =  { 2.47, 2.77, 3.06, 3.35} ;
    private static final double lowVelocities[] = { 60, 63, 67, 72 } ;

    private static final double medHood = 25 ;
    private static final double medDist[] = { 3.69, 4.02, 4.30  } ;
    private static final double medVelocities[] = { 66, 69, 72 };

    private static final double highPlusHood = 35 ;
    private static final double highPlusDist[] = { 4.60, 4.92, 5.24 } ;
    private static final double highPlusVelocity[] = { 68, 71, 83 } ;
    
    public ArcShooterTuning() {
        super("arc") ;
    }

    protected void initData() {
        addOneHood(touchingHood, touchingDist, touchingVelocities) ;
        addOneHood(lowHood, lowDist, lowVelocities) ;
        addOneHood(medHood, medDist, medVelocities) ;
        addOneHood(highPlusHood, highPlusDist, highPlusVelocity) ;
    }
}
