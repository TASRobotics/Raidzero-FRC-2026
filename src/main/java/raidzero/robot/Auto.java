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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import raidzero.lib.Util;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.intake.Extender;
import raidzero.robot.subsystems.intake.Intake;

public class Auto {
    private final Swerve swerve;
    private final Intake intake;
    private final Extender extender;
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
    public Auto(Swerve swerve, Intake intake, Extender extender) {
        this.swerve = swerve;
        this.intake = intake;
        this.extender = extender;
        autoloadPaths();
    }

    /**
     * Scans the deploy/choreo directory and loads all .traj files into memory
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
     * Gets a loaded path by name
     */
    public PathPlannerPath get(String name) {
        if (!pathCache.containsKey(name)) {
            DriverStation.reportError("Auto: Path '" + name + "' not found in cache!", false);
            return null;
        }
        return pathCache.get(name);
    }

    /**
     * Updates the constraints for pathfinding commands
     */
    public Command setConstraints(PathConstraints newConstraints) {
        return Commands.runOnce(() -> this.currentConstraints = newConstraints);
    }

    /**
     * Pathfinds to the starting pose of the given path, then follows it
     *
     * @param pathName The name of the path
     * @return A {@link DeferredCommand} that pathfinds to the start of the path, then follows it
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
     * Follows the given path using Pathplanner's {@link AutoBuilder#followPath}
     *
     * @param pathName The name of the path to follow
     * @return A {@link Command} that follows the given path
     */
    public Command follow(String pathName) {
        return AutoBuilder.followPath(get(pathName));
    }

    /**
     * Builds an autonomous routine by first pathfinding to the starting pose of the given path, then executing the provided logic
     *
     * @param startPathName The name of the first path in the routine
     * @param autoLogic A {@link Command} {@link Supplier} that provides the main logic of the routine
     * @return A {@link DeferredCommand} that executes the routine
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
        }, Set.of(swerve, intake, extender));
    }

    /**
     * Flips the given pose across the field if the current alliance is Red
     *
     * @param bluePose The pose with respect to the blue alliance
     * @return A flipped {@link Pose2d} if the alliance is Red, otherwise the original pose
     */
    private Pose2d getAlliancePose(Pose2d bluePose) {
        return Util.isRedAlliance() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    // --- AUTONOMOUS ROUTINES ---

    /**
     * A test auto
     *
     * @return A {@link Command} for the depot 1 cycle test auto
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

    /**
     * Auto that starts on the left trench and performs two cycles
     *
     * @return A {@link Command} for the left trench 2 cycle auto
     */
    public Command getLeft2Cycle() {
        return build(
            "LeftMidStart", () -> Commands.sequence(
                Commands.deadline(
                    follow("LeftMidStart"),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.go()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        extender.extend()
                    )
                ),

                Commands.deadline(
                    follow("LeftMidTrenchShoot"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(3.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("LeftMidCollect"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                ),

                follow("LeftMidCollectTrenchOut"),
                Commands.deadline(
                    follow("LeftMidShoot"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("LeftShootToMid"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                )
            )
        );
    }

    /**
     * Auto that starts on the right trench and performs one cycle with the outpost
     *
     * @return A {@link Command} for the right trench 1 cycle outpost auto
     */
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
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("OutpostToMid"),
                    Commands.sequence(
                        Commands.waitSeconds(2),
                        intake.go()
                    )
                )
            )
        );
    }

    /**
     * Auto that starts on the right trench and performs two cycles with the outpost
     *
     * @return A {@link Command} for the right trench 2 cycle outpost auto
     */
    public Command getRightOutpostDoubleAuto() {
        return build(
            "RightMidStartFast", () -> Commands.sequence(
                // Drive the first path as the leader
                Commands.deadline(
                    follow("RightMidStartFast"),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.go()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.3),
                        extender.extend()
                    )
                ),

                Commands.deadline(
                    follow("RightMidTrenchShoot"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(3.0),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("RightMidCollect"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                ),

                follow("RightMidCollectTrenchIn"),

                // Drive and stop intake simultaneously
                Commands.deadline(
                    follow("RightMidToOutpostFast"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("OutpostToMidFast"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                )
            )
        );
    }

    /**
     * Auto that starts on the right trench and performs two cycles
     *
     * @return A {@link Command} for the right trench 2 cycle auto
     */
    public Command getRight2Cycle() {
        return build(
            "RightMidStartFast", () -> Commands.sequence(
                Commands.deadline(
                    follow("RightMidStartFast"),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        intake.go()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.3),
                        extender.extend()
                    )
                ),

                Commands.deadline(
                    follow("RightMidTrenchShoot"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(3.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("RightMidCollect"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                ),

                follow("RightMidCollectTrenchOut"),
                Commands.deadline(
                    follow("RightMidShoot"),
                    Commands.sequence(
                        Commands.waitSeconds(1.75),
                        intake.stop()
                    )
                ),

                Commands.deadline(
                    Commands.waitSeconds(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.5),
                        Commands.parallel(
                            extender.autoAgitate().repeatedly(),
                            intake.stop()
                        )
                    )
                ),

                Commands.deadline(
                    follow("RightShootToMid"),
                    Commands.sequence(
                        extender.extend().withTimeout(1),
                        intake.go()
                    )
                )
            )
        );
    }
}