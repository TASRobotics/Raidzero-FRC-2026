package raidzero.lib;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {
    /**
     * Checks if the current alliance is Red. If the alliance is not yet determined, returns false by default.
     * 
     * @return True if the current alliance is Red, false otherwise (including if the alliance is not yet determined)
     */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }

        return false;
    }
}
