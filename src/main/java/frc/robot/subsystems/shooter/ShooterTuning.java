package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTuning {

    public class ShooterParams {
        public double dist ;
        public double hood ;
        public double velocity ;

        public ShooterParams(double d, double h, double v) {
            dist = d ;
            hood = h ;
            velocity = v ;
        }
    } ;

    private class OneSettings {
        public double hood_ ;
        public double min_dist_ ;
        public double max_dist_ ;
        public InterpolatingDoubleTreeMap map_ ;

        public OneSettings(double h, double[] d, double[] v) {
            hood_ = h ;
            map_ = new InterpolatingDoubleTreeMap() ;
            for(var i = 0 ; i < d.length ; i++) {
                map_.put(d[i], v[i]) ;
            }

            min_dist_ = d[0] ;
            max_dist_ = d[d.length - 1] ;}
    }
    
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

    private List<OneSettings> settings_ = new ArrayList<OneSettings>() ;
    private int lastHoodIndex_ ;

    public ShooterTuning() {
        try {
            initData() ;
        }
        catch(Exception ex) {
        }

        lastHoodIndex_ = -1 ;
    }

    public ShooterParams getShooterParams(double dist) {
        int h = getHoodIndex(dist) ;
        var ret = new ShooterParams(dist, settings_.get(h).hood_, settings_.get(h).map_.get(dist)) ;
        Logger.recordOutput("ShooterTuning/hood", ret.hood);
        Logger.recordOutput("ShooterTuning/velocity", ret.velocity);
        return ret ;
    }

    private int getHoodIndex(double dist) {
        for(int i = 0 ; i < settings_.size() ; i++) {
            if (dist < settings_.get(i).max_dist_) {
                return processHysteresis(i) ;
            }
        }

        return -1 ;
    }

    private int processHysteresis(int newIndex) {
        if (lastHoodIndex_ == -1) {
            lastHoodIndex_ = newIndex ;
            return newIndex ;
        }

        if (newIndex == lastHoodIndex_) {
            return newIndex ;
        }

        if (newIndex > lastHoodIndex_) {
            // moving up, add some hysteresis
            if (settings_.get(newIndex).min_dist_ - settings_.get(lastHoodIndex_).min_dist_ < 0.5) {
                return lastHoodIndex_ ;
            }
        }
        else {
            // moving down, add some hysteresis
            if (settings_.get(lastHoodIndex_).min_dist_ - settings_.get(newIndex).min_dist_ < 0.5) {
                return lastHoodIndex_ ;
            }
        }

        lastHoodIndex_ = newIndex ;
        return newIndex ;
    }


    private void initData() {
        addOneHood(touchingHood, touchingDist, touchingVelocities) ;
        addOneHood(lowHood, lowDist, lowVelocities) ;
        addOneHood(medHood, medDist, medVelocities) ;
        addOneHood(highPlusHood, highPlusDist, highPlusVelocity) ;
    }

    private void addOneHood(double hood, double[] dist, double[] vels)  {
        if (dist.length != vels.length) {
            throw new RuntimeException("invalid data, dist and vel data should be the same size") ;
        }

        if (dist.length < 2) {
            throw new RuntimeException("invalid data, each hood value should contain two entries") ;
        }

        settings_.add(new OneSettings(hood, dist, vels));

        settings_.sort((s1, s2) -> {
            return Double.compare(s1.hood_, s2.hood_);
        }) ;
    }
}
