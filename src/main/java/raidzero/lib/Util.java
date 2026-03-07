package raidzero.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {
    /**
     * Checks if the robot is on the blue alliance
     *
     * @return true if the robot is on the blue alliance, false otherwise
     */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
    }
}
