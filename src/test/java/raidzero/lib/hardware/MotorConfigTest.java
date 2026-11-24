package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import raidzero.lib.hardware.MotorConfig.Controller;
import raidzero.lib.hardware.MotorConfig.Direction;
import raidzero.lib.hardware.MotorConfig.Follower;
import raidzero.lib.hardware.MotorConfig.Gearing;
import raidzero.lib.hardware.MotorConfig.MotorModel;
import raidzero.lib.hardware.MotorConfig.NeutralMode;

class MotorConfigTest {
    private static MotorConfig motor(Controller controller, MotorModel model, List<Follower> followers) {
        return new MotorConfig(
                new CanId(1, CanBus.RIO),
                controller,
                model,
                Direction.COUNTERCLOCKWISE_POSITIVE,
                NeutralMode.BRAKE,
                CurrentLimits.stator(Amps.of(40)).supply(Amps.of(30)),
                new Gearing(10),
                followers);
    }

    @Test
    void rejectsMotorTheControllerCantDrive() {
        assertThrows(IllegalArgumentException.class, () -> motor(Controller.TALON_FX, MotorModel.MINION, List.of()));
        assertThrows(
                IllegalArgumentException.class, () -> motor(Controller.TALON_FXS, MotorModel.KRAKEN_X60, List.of()));
    }

    @Test
    void rejectsDuplicateCanIds() {
        assertThrows(
                IllegalArgumentException.class,
                () -> motor(Controller.TALON_FX, MotorModel.KRAKEN_X60, List.of(new Follower(1, false))));
        assertThrows(
                IllegalArgumentException.class,
                () -> motor(
                        Controller.TALON_FX,
                        MotorModel.KRAKEN_X60,
                        List.of(new Follower(2, false), new Follower(2, true))));
    }

    @Test
    void countsFollowersAndCopiesTheList() {
        List<Follower> followers = new ArrayList<>(List.of(new Follower(2, true)));
        MotorConfig config = motor(Controller.TALON_FX, MotorModel.KRAKEN_X60, followers);
        followers.add(new Follower(3, false));
        assertEquals(2, config.motorCount());
    }

    @Test
    void rejectsBadValues() {
        assertThrows(IllegalArgumentException.class, () -> new CanId(63, CanBus.RIO));
        assertThrows(IllegalArgumentException.class, () -> CurrentLimits.stator(Amps.of(0))
                .supply(Amps.of(30)));
        assertThrows(IllegalArgumentException.class, () -> new CanBus(" "));
        assertThrows(IllegalArgumentException.class, () -> new Gearing(0));
        assertThrows(IllegalArgumentException.class, () -> Gearing.stages(42, 10, 42));
    }

    @Test
    void gearingMultipliesStages() {
        assertEquals(
                (42.0 / 10.0) * (42.0 / 14.0), Gearing.stages(42, 10, 42, 14).reduction(), 1e-9);
    }
}
