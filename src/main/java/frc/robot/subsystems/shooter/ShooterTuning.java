package frc.robot.subsystems.shooter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.core.JsonParser;
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
        public double[] dists_ ;
        public double[] vels_ ;
        public InterpolatingDoubleTreeMap map_ ;

        public OneHoodTuning(double h, double[] d, double[] v) {
            hood_ = h ;
            dists_ = d ;
            vels_ = v ;
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

    public void reset() {
        lastHoodIndex_ = -1 ;
    }

    public String getName() {
        return name_ ;
    }

    private void readTuningData(String filename) {
        try {
            File file = new File(Filesystem.getDeployDirectory(), filename) ;
            ObjectMapper mapper = new ObjectMapper() ;
            mapper.enable(JsonParser.Feature.ALLOW_COMMENTS) ;
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

            System.out.println("ShooterTuning: successfully read tuning data from " + filename) ;
        } catch (Exception e) {
            System.err.println("ShooterTuning: failed to read tuning data from " + filename + ": " + e.getMessage()) ;
        }
    }

    private double doBetterInterpolate(OneHoodTuning tuning, double dist, boolean last) {
        double[] d = tuning.dists_ ;
        double[] v = tuning.vels_ ;

        // Below the lowest data point - extrapolate using the first two points
        if (dist <= d[0]) {
            double slope = (v[1] - v[0]) / (d[1] - d[0]) ;
            return v[0] + slope * (dist - d[0]) ;
        }

        // Above the highest data point - extrapolate using the last two points
        int n = d.length ;

        if (dist >= d[n - 1]) {
            // If the last hood index, dont interpolate outwards.
            if (last) {
                Logger.recordOutput("NotInterpolatingEnd", true);
                return v[n - 1];
            } else {
                Logger.recordOutput("NotInterpolatingEnd", false);
            }

            double slope = (v[n - 1] - v[n - 2]) / (d[n - 1] - d[n - 2]) ;
            return v[n - 1] + slope * (dist - d[n - 1]) ;
        }

        // Within range - use the interpolating tree map
        return tuning.map_.get(dist) ;
    }

    public ShooterParams getShooterParams(double dist) {
        lastHoodIndex_ = -1 ;
        int h = getHoodIndex(dist) ;
        double vel = doBetterInterpolate(settings_.get(h), dist, h == settings_.size() - 1) ;
        var ret = new ShooterParams(dist, settings_.get(h).hood_, vel) ;
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
            System.out.println("ShooterTuning: setting initial hood index to " + newIndex + " for distance " + dist) ;
            return newIndex ;
        }

        if (newIndex == lastHoodIndex_ + 1) {
            // We moved to the next hood index
            // Stay at the old hood index until we exceed its max distance plus hysteresis
            if (dist <= settings_.get(lastHoodIndex_).max_dist_ + HYSTERESIS_DIST) {
                System.out.println("ShooterTuning: 1 staying at hood index " + lastHoodIndex_ + " for distance " + dist + " due to hysteresis") ;
                return lastHoodIndex_ ;
            }
        }
        else if (newIndex == lastHoodIndex_ - 1) {
            // We moved to the previous hood index
            if (dist > settings_.get(lastHoodIndex_).min_dist_ - HYSTERESIS_DIST) {
                System.out.println("ShooterTuning: 2 staying at hood index " + lastHoodIndex_ + " for distance " + dist + " due to hysteresis") ;
                return lastHoodIndex_ ;
            }
        }

        System.out.println("ShooterTuning: moving from hood index " + lastHoodIndex_ + " to " + newIndex + " for distance " + dist) ;
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
