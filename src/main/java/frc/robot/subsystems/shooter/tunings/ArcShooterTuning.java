package frc.robot.subsystems.shooter.tunings;

public class ArcShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final double touchingDist[] = { 1.616 , 1.86, 2.18} ;
    private static final double touchingVelocities[] = {  55.0, 58.0, 65.0 } ;
    private static final double touchingHangtime[] = { 1.8, 1.8, 1.8 } ;

    private static final double lowHood = 15 ;
    private static final double lowDist[] =  { 2.47, 2.77, 3.06, 3.35} ;
    private static final double lowVelocities[] = { 62, 64, 68, 73 } ;
    private static final double lowHangtime[] = { 1.8, 1.8, 1.8, 1.8 } ;

    private static final double medHood = 25 ;
    private static final double medDist[] = { 3.69, 4.02, 4.30  } ;
    private static final double medVelocities[] = { 70, 72, 75 };
    private static final double medHangtime[] = { 1.8, 1.8, 1.8 } ;
    
    private static final double highPlusHood = 35 ;
    private static final double highPlusDist[] = { 4.60, 4.92, 5.24 } ;
    private static final double highPlusVelocity[] = { 68, 71, 83 } ;
    private static final double highPlusHangtime[] = { 1.8, 1.8, 1.8 } ;
    
    public ArcShooterTuning() {
        super("arc") ;
    }

    protected void initData() {
        addOneHood(touchingHood, touchingDist, touchingVelocities, touchingHangtime) ;
        addOneHood(lowHood, lowDist, lowVelocities, lowHangtime) ;
        addOneHood(medHood, medDist, medVelocities, medHangtime) ;
        addOneHood(highPlusHood, highPlusDist, highPlusVelocity, highPlusHangtime) ;
    }
}
