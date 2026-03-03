package frc.robot.subsystems.shooter.tunings;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public abstract class ShooterTuning {

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

    private final double HYSTERESIS_DIST = 0.06; // in meters, about a foot of hysteresis when
    private List<OneSettings> settings_ = new ArrayList<OneSettings>() ;
    private int lastHoodIndex_ ;
    private String name_ ;

    public ShooterTuning(String name) {
        this.name_ = name ; 
        initData() ;
        lastHoodIndex_ = -1 ;
    }

    public String getName() {
        return name_ ;
    }

    protected abstract void initData() ;

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
                return processHysteresis(dist, i) ;
            }
        }

        return settings_.size() - 1 ;
    }

    private int processHysteresis(double dist, int newIndex) {
        if (lastHoodIndex_ == -1) {
            lastHoodIndex_ = newIndex ;
            return newIndex ;
        }

        if (newIndex == lastHoodIndex_ + 1) {
            // We moved to the next hood index
            // Stay at the old hood index until we exceed its max distance plus hysteresis
            if (dist <= settings_.get(lastHoodIndex_).max_dist_ + HYSTERESIS_DIST) {
                return lastHoodIndex_ ;
            }
        }
        else if (newIndex == lastHoodIndex_ - 1) {
            // We moved to the previous hood index
            if (dist > settings_.get(lastHoodIndex_).min_dist_ - HYSTERESIS_DIST) {
                return lastHoodIndex_ ;
            }
        }

        lastHoodIndex_ = newIndex ;
        return newIndex ;
    }

    protected void addOneHood(double hood, double[] dist, double[] vels)  {
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
