package raidzero.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import raidzero.lib.Util;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.intake.Extender;
import raidzero.robot.subsystems.intake.Intake;
import raidzero.robot.subsystems.intake.Spindexer;
import raidzero.robot.subsystems.shooter.Feeder;

public class Auto {
    private final Swerve swerve;
    private final Intake intake;
    private final Extender extender;
    private final Spindexer spindexer;
    private final Feeder feeder;
    private final Map<String, PathPlannerPath> pathCache = new HashMap<>();

    private PathConstraints currentConstraints = SLOW_CONSTRAINTS;

    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        3.0, 2.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    public static final PathConstraints SLOW_CONSTRAINTS = new PathConstraints(
        1.0, 1.2,
        Units.degreesToRadians(360),
        Units.degreesToRadians(480)
    );

    /**
     * Constructs a {@link Auto} instance
     *
     * @param swerve A {@link Swerve} instance
     */
    public Auto(Swerve swerve, Intake intake, Extender extender, Spindexer spindexer, Feeder feeder) {
        this.swerve = swerve;
        this.intake = intake;
        this.extender = extender;
        this.spindexer = spindexer;
        this.feeder = feeder;
        autoloadPaths();
    }

    /**
     * Scans the deploy/choreo directory and loads all .traj files into RAM.
     */
    private void autoloadPaths() {
        File choreoDir = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] files = choreoDir.listFiles((dir, name) -> name.endsWith(".traj"));

        if (files == null || files.length == 0) {
            DriverStation.reportWarning("Auto: No Choreo paths found in deploy/choreo!", false);
            return;
        }

        for (File file : files) {
            String name = file.getName().replace(".traj", "");
            try {
                pathCache.put(name, PathPlannerPath.fromChoreoTrajectory(name));
            } catch (Exception e) {
                DriverStation.reportError("Auto: Failed to load " + name + ": " + e.getMessage(), e.getStackTrace());
            }
        }
    }

    /**
     * Safely retrieves a path from the cache.
     */
    public PathPlannerPath get(String name) {
        if (!pathCache.containsKey(name)) {
            DriverStation.reportError("Auto: Path '" + name + "' not found in cache!", false);
            return null;
        }
        return pathCache.get(name);
    }

    /**
     * Updates the constraints used for all subsequent OTF (Pathfinding) calls.
     */
    public Command setConstraints(PathConstraints newConstraints) {
        return Commands.runOnce(() -> this.currentConstraints = newConstraints);
    }

    /**
     * Abstracted pathfinding helper.
     * Uses defer to ensure it uses the constraints active at the time of execution.
     */
    public Command otf(String pathName) {
        return Commands.defer(
            () -> Commands.sequence(
                AutoBuilder.pathfindToPoseFlipped(
                    get(pathName).getStartingHolonomicPose().orElse(new Pose2d()), SLOW_CONSTRAINTS,
                    get(pathName).getIdealStartingState().velocity()
                ),
                follow(pathName)
            ),
            Set.of(swerve)
        );
    }

    /**
     * Standard follow command (no pathfinding).
     */
    public Command follow(String pathName) {
        return AutoBuilder.followPath(get(pathName));
    }

    /**
     * Master wrapper for Auto Routines.
     */
    private Command build(String startPathName, Supplier<Command> autoLogic) {
        return Commands.defer(() -> {
            Pose2d blueStartPose = get(startPathName).getStartingHolonomicPose().orElse(new Pose2d());
            Pose2d initialPose = getAlliancePose(blueStartPose);

            return Commands.sequence(
                Commands.runOnce(() -> {
                    if (RobotBase.isSimulation() || startPathName.equals("StraightLine") || startPathName.equals("RightRightAngle"))
                        swerve.resetPose(initialPose);
                }),
                autoLogic.get()
            );
        }, Set.of(swerve, intake, extender, spindexer, feeder));
    }

    /**
     * Flips the given pose across the field if the current alliance is Red
     *
     * @param bluePose The pose with respect to the blue alliance
     * @return The pose transformed for the current alliance (flipped if Red,
     *         unchanged if Blue)
     */
    private Pose2d getAlliancePose(Pose2d bluePose) {
        return Util.isRedAlliance() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    // --- AUTONOMOUS ROUTINES ---

    /**
     * A test auto
     *
     * @return A {@link Command} for the depot 1 cycle auto
     */
    public Command getDepot1Cycle() {
        return build(
            "DepotCross", () -> Commands.sequence(
                follow("DepotCross"),
                Commands.waitSeconds(0.1),
                follow("DepotClimb")
            )
        );
    }

    public Command getLeft2Cycle() {
        return build(
            "LeftMidStart", () -> Commands.sequence(
                follow("LeftMidStart"),
                Commands.waitSeconds(0.5),
                otf("LeftMidClimb")
            )
        );
    }

    // public Command getLeftMidAuto() {
    // return build("LeftMidStart", () -> Commands.sequence(
    // otf("LeftMidStart"),
    // swerve.randomIntake(2),
    // otf("LeftMidShoot"),
    // Commands.waitSeconds(3),
    // follow("LeftMidCollect"),
    // swerve.randomIntake(2),
    // otf("LeftMidShoot"),
    // Commands.waitSeconds(3),
    // otf("LeftMidClimb")
    // ));
    // }

    // public Command getLeftDepotAuto() {
    // return build("LeftMidStart", () -> Commands.sequence(
    // otf("LeftMidStart"),
    // swerve.randomIntake(2),
    // otf("LeftMidShoot"),
    // Commands.waitSeconds(3),
    // follow("LeftMidCollect"),
    // swerve.randomIntake(2),
    // otf("MidToDepot"),
    // follow("DepotCross"),
    // follow("DepotClimb")
    // ));
    // }

    // public Command getRightMidAuto() {
    // return build("RightMidStart", () -> Commands.sequence(
    // otf("RightMidStart"),
    // swerve.randomIntake(2),
    // otf("RightMidShoot"),
    // Commands.waitSeconds(3),
    // follow("RightMidCollect"),
    // swerve.randomIntake(2),
    // otf("RightMidShoot"),
    // Commands.waitSeconds(3),
    // follow("RightMidCollect"),
    // swerve.randomIntake(2),
    // otf("RightMidClimb")
    // ));
    // }

    public Command getRightOutpostAuto() {
        return build(
            "RightMidStart", () -> Commands.sequence(
                // Drive the first path as the leader
                Commands.deadline(
                    follow("RightMidStart"),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.go()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        extender.extend()
                    )
                ),

                follow("RightMidTrenchIn"),

                // Drive and stop intake simultaneously
                Commands.deadline(
                    follow("RightMidToOutpost"),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        intake.stop()
                    ),
                    feeder.go()
                )
            )
        );
    }
}