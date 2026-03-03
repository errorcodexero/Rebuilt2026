package frc.robot.subsystems.shooter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;

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

    public static class OneTuningValue {
        public final double dist_ ;
        public final double vel_ ;

        public OneTuningValue(double d, double v) {
            dist_ = d ;
            vel_ = v ;
        }
    }

    private class OneHoodTuning {
        public double hood_ ;
        public double min_dist_ ;
        public double max_dist_ ;
        public InterpolatingDoubleTreeMap map_ ;

        public OneHoodTuning(double h, double[] d, double[] v) {
            hood_ = h ;
            map_ = new InterpolatingDoubleTreeMap() ;
            for(var i = 0 ; i < d.length ; i++) {
                map_.put(d[i], v[i]) ;
            }

            min_dist_ = d[0] ;
            max_dist_ = d[d.length - 1] ;}
    }


    private final double HYSTERESIS_DIST = 0.06; // in meters, about a foot of hysteresis when
    private List<OneHoodTuning> settings_ = new ArrayList<OneHoodTuning>() ;
    private int lastHoodIndex_ ;
    private String name_ ;

    public ShooterTuning(String name) {
        this.name_ = name ; 
        lastHoodIndex_ = -1 ;
        readTuningData("tuning/" + name + ".json") ;
    }

    public String getName() {
        return name_ ;
    }

    private void readTuningData(String filename) {
        try {
            File file = new File(Filesystem.getDeployDirectory(), filename) ;
            ObjectMapper mapper = new ObjectMapper() ;
            JsonNode root = mapper.readTree(file) ;

            JsonNode dataArray = root.get("data") ;
            if (dataArray == null || !dataArray.isArray()) {
                return ;
            }

            settings_.clear() ;

            for (JsonNode hoodNode : dataArray) {
                double hood = hoodNode.get("hood").asDouble() ;
                JsonNode pointsArray = hoodNode.get("points") ;

                if (pointsArray == null || !pointsArray.isArray() || pointsArray.size() < 2) {
                    continue ;
                }

                OneTuningValue[] values = new OneTuningValue[pointsArray.size()] ;
                for (int i = 0 ; i < pointsArray.size() ; i++) {
                    JsonNode pt = pointsArray.get(i) ;
                    double distance = pt.get("distance").asDouble() ;
                    double velocity = pt.get("velocity").asDouble() ;
                    values[i] = new OneTuningValue(distance, velocity) ;
                }

                addOneHood(hood, values) ;
            }
        } catch (Exception e) {
            System.err.println("ShooterTuning: failed to read tuning data from " + filename + ": " + e.getMessage()) ;
        }
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

    protected void addOneHood(double hood, OneTuningValue[] values) {

        if (values.length < 2) {
            throw new RuntimeException("invalid data, each hood value should contain two entries") ;
        }

        double [] dist = new double[values.length] ;
        double [] vels = new double[values.length] ;

        for(int i = 0 ; i < values.length ; i++) {
            var v = values[i] ;
            if (v.dist_ < 0 || v.vel_ < 0) {
                throw new RuntimeException("invalid data, distance and velocity should be positive") ;
            }
            dist[i] = v.dist_ ;
            vels[i] = v.vel_ ;
        }   
        settings_.add(new OneHoodTuning(hood, dist, vels));

        settings_.sort((s1, s2) -> {
            return Double.compare(s1.hood_, s2.hood_);
        }) ;
    }
}
