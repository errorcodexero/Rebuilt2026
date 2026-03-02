package frc.robot.subsystems.shooter.tunings;

public class ArcShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final double touchingDist[] = { 1.37 , 2.13} ;
    private static final double touchingVelocities[] = {  55.0, 60.0 } ;

    private static final double lowHood = 15 ;
    private static final double lowDist[] =  {2.16, 2.76 } ;
    private static final double lowVelocities[] = { 53.0, 60.0} ;

    private static final double medHood = 25 ;
    private static final double medDist[] = { 3.937, 4.15 } ;
    private static final double medVelocities[] = { 65.0, 67.0 };

    private static final double highPlusHood = 50 ;
    private static final double highPlusDist[] = { 4.47, 5.2 } ;
    private static final double highPlusVelocity[] = { 68, 62} ;
    
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
