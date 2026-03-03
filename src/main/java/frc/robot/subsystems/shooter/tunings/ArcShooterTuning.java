package frc.robot.subsystems.shooter.tunings;

public class ArcShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final OneTuningValue touchingValues[] = {
        new OneTuningValue(1.616, 55.0),                                    // Test Mode
        new OneTuningValue(1.86, 58.0),                                     // Test Mode
        new OneTuningValue(2.18, 65.0)                                      // Test Mode
    } ;
    
    private static final double lowHood = 15 ;
    private static final OneTuningValue lowValues[] = {
        new OneTuningValue(2.47, 62.0),                                   // Test Mode
        new OneTuningValue(2.77, 64.0),                                   // Test Mode
        new OneTuningValue(3.06, 68.0),                                   // Test Mode
        new OneTuningValue(3.35, 73.0)                                    // Test Mode
    } ;

    private static final double medHood = 25 ;
    private static final OneTuningValue medValues[] = {
        new OneTuningValue(3.69, 70.0),                                   // Test Mode
        new OneTuningValue(4.02, 72.0),                                   // Test Mode
        new OneTuningValue(4.30, 75.0)                                    // Test Mode
    } ;

    private static final double highPlusHood = 35 ;
    private static final OneTuningValue highPlusValues[] = {
        new OneTuningValue(4.60, 68.0),                                    // Test Mode
        new OneTuningValue(4.92, 71.0),                                    // Test Mode
        new OneTuningValue(5.24, 83.0)                                     // Test Mode
    } ;
    
    public ArcShooterTuning() {
        super("arc") ;
    }

    protected void initData() {
        addOneHood(touchingHood, touchingValues) ;
        addOneHood(lowHood, lowValues) ;
        addOneHood(medHood, medValues) ;
        addOneHood(highPlusHood, highPlusValues) ;
    }
}
