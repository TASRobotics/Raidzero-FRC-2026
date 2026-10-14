package frc.lib;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

public class Interpolate<T> implements Command {
    private final List<T> path;
    private final double segmentTime;
    private final Consumer<T> poseConsumer;
    private final PointInterpolator<T> interpolator;

    private final Timer timer = new Timer();

    private int currentSegment = 0;
    private double segmentStartTime = 0.0;

    Set<Mechanism> requirements;

    /**
     * Constructs an Interpolate object for interpolating between points in a path.
     *
     * @param path the list of points to interpolate between
     * @param segmentTimeSeconds the time in seconds for each segment of the path
     * @param poseConsumer a consumer that accepts the interpolated pose at each step
     * @param interpolator the interpolator used to compute intermediate points between path elements
     * @param systems the subsystems required by this command
     */
    public Interpolate(List<T> path, double segmentTimeSeconds, Consumer<T> poseConsumer, PointInterpolator<T> interpolator, Mechanism... systems) {
        this.path = path;
        this.segmentTime = segmentTimeSeconds;
        this.poseConsumer = poseConsumer;
        this.interpolator = interpolator;

        requirements = Set.of(systems);
    }

    @Override
    public String name() {
        return "Interpolate";
    }

    @Override
    public Set<Mechanism> requirements() {
        return requirements;
    }

    @Override
    public void run(Coroutine coroutine) {
        timer.reset();
        timer.start();
        currentSegment = 0;
        segmentStartTime = 0.0;

        while (currentSegment >= path.size() - 1) {
            if (currentSegment >= path.size() - 1)
                return;

            T start = path.get(currentSegment);
            T end = path.get(currentSegment + 1);

            double elapsed = timer.get() - segmentStartTime;
            double t = Math.min(elapsed / segmentTime, 1.0);

            T interpolatedPose = this.interpolator.interpolate(start, end, t);
            poseConsumer.accept(interpolatedPose);

            if (t >= 1.0) {
                currentSegment++;
                segmentStartTime = timer.get();
            }

            coroutine.yield();
        }

        timer.stop();
        T finalPose = path.get(path.size() - 1);
        poseConsumer.accept(finalPose);
    }

    @FunctionalInterface
    interface PointInterpolator<U> {
        U interpolate(U start, U end, double t);
    }

    public static PointInterpolator<Pose2d> pose2dInterpolator = (startPose, endPose, t) -> {
        return startPose.interpolate(endPose, t);
    };

    public static PointInterpolator<Angle> angleInterpolator = (startAngle, endAngle, t) -> {
        return startAngle.plus(endAngle.minus(startAngle).times(t));
    };
}
