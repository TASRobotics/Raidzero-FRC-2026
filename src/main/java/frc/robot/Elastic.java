package frc.robot;

import org.wpilib.driverstation.RobotState;

import frc.lib.ElasticLib;
import frc.lib.Util;

public class Elastic {
    private static boolean isBlueAlliance = true;

    /**
     * Selects the setup tab in Elastic
     */
    public static void init() {
        ElasticLib.selectTab("Setup");
    }

    /**
     * Selects the auton tab in Elastic
     */
    public static void initAuton() {
        ElasticLib.selectTab("Auton");
    }

    /**
     * Selects the teleop tab in Elastic based on alliance color
     */
    public static void initTeleop() {
        if (isBlueAlliance) {
            ElasticLib.selectTab("BTeleop");
        } else {
            ElasticLib.selectTab("RTeleop");
        }
    }

    /**
     * Checks the alliance color and sets the isBlueAlliance variable accordingly
     */
    public static void checkForAlliance() {
        if (!RobotState.isDSAttached()) {
            return;
        }

        if (Util.isBlueAlliance()) {
            isBlueAlliance = true;
        } else {
            isBlueAlliance = false;
        }
    }
}
