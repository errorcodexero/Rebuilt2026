package frc.robot.subsystems.shooter.tunings;

public class FlatShooterTuning extends ShooterTuning {
    
    // Tested Distances
    private static final double touchingHood = 5 ;
    private static final OneTuningValue touchingValues[] = {
        new OneTuningValue(1.616, 55.0),                                    // Test Mode
        new OneTuningValue(1.86, 58.0),                                     // Test Mode
        new OneTuningValue(2.18, 65.0)                                      // Test Mode
    } ; 

    private static final double lowHood = 35 ;
    private static final OneTuningValue lowValues[] = {
        new OneTuningValue(2.47, 62.0),                                   // Test Mode
        new OneTuningValue(2.77, 64.0),                                   // Test Mode
        new OneTuningValue(3.06, 68.0),                                   // Test Mode
        new OneTuningValue(3.35, 73.0)                                    // Test Mode
    } ;

    private static final double medHood = 45 ;
    private static final OneTuningValue medValues[] = {
        new OneTuningValue(4.0, 58.0),                                    // Test Mode
        new OneTuningValue(4.31, 61.0),                                   // Test Mode
        new OneTuningValue(4.93, 66.0)                                    // Test Mode
    } ;

    public FlatShooterTuning() {
        super("flat") ;
    }

    protected void initData() {
        addOneHood(touchingHood, touchingValues) ;
        addOneHood(lowHood, lowValues) ;
        addOneHood(medHood, medValues) ;
    }    
}
