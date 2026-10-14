package frc.lib;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import java.util.Optional;

public class Util {
    /**
     * Checks if the robot is on the blue alliance
     *
     * @return true if the robot is on the blue alliance, false otherwise
     */
    public static boolean isBlueAlliance() {
        return MatchState.getAlliance().orElse(Alliance.RED) == Alliance.BLUE;
    }

    /**
     * Checks if the current alliance is Red. If the alliance is not yet determined, returns false by default.
     *
     * @return True if the current alliance is Red, false otherwise (including if the alliance is not yet determined)
     */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = MatchState.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == Alliance.RED;
        }

        return false;
    }
}
