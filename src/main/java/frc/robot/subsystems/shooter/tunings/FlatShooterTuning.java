package frc.robot.subsystems.shooter.tunings;

public class FlatShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final double touchingDist[] = { 1.616 , 1.86, 2.18} ;
    private static final double touchingVelocities[] = {  55.0, 58.0, 65.0 } ;

    private static final double lowHood = 35 ;
    private static final double lowDist[] =  { 2.47, 2.78, 3.08, 3.40 } ;
    private static final double lowVelocities[] = { 52, 54, 56, 59} ;

    private static final double medHood = 45 ;
    private static final double medDist[] = { 4.0, 4.31, 4.93} ;
    private static final double medVelocities[] = { 58, 61, 66 };

    public FlatShooterTuning() {
        super("flat") ;
    }

    protected void initData() {
        addOneHood(touchingHood, touchingDist, touchingVelocities) ;
        addOneHood(lowHood, lowDist, lowVelocities) ;
        addOneHood(medHood, medDist, medVelocities) ;
    }    
}
