package raidzero.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {

    public static class Field {
        public static final Translation2d GOAL_LOCATION_BLUE = new Translation2d(4.615, 4.035);
        public static final Translation2d GOAL_LOCATION_RED = new Translation2d(7.2898, 4.035);
    }

    public static class Shooter {
        public static final InterpolatingDoubleTreeMap PITCH_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap TOF_TABLE = new InterpolatingDoubleTreeMap();

        static {
            // --- PITCH TABLE ---
            PITCH_TABLE.put(80.0, 75.0);
            PITCH_TABLE.put(120.0, 60.0);
            PITCH_TABLE.put(160.0, 45.0);
            PITCH_TABLE.put(200.0, 35.0);
            PITCH_TABLE.put(250.0, 30.0);

            // --- RPM TABLE ---
            RPM_TABLE.put(80.0, 2500.0);
            RPM_TABLE.put(120.0, 2800.0);
            RPM_TABLE.put(160.0, 3200.0);
            RPM_TABLE.put(200.0, 3700.0);
            RPM_TABLE.put(250.0, 4500.0);

            // --- TIME OF FLIGHT TABLE ---
            TOF_TABLE.put(80.0, 0.45);
            TOF_TABLE.put(150.0, 0.75);
            TOF_TABLE.put(250.0, 1.10);
        }
    }

    public static class Turret {

        public static final Translation2d TURRET_OFFSET = new Translation2d(0.3, 0.0); // offset from robot center

    }
}
