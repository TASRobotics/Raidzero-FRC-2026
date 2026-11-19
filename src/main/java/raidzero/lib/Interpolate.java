package raidzero.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.Consumer;

public class Interpolate<T> extends Command {
    private final List<T> path;
    private final double segmentTime;
    private final Consumer<T> poseConsumer;
    private final PointInterpolator<T> interpolator;

    private final Timer timer = new Timer();

    private int currentSegment = 0;
    private double segmentStartTime = 0.0;

    /**
     * Constructs an Interpolate object for interpolating between points in a path.
     *
     * @param path the list of points to interpolate between
     * @param segmentTimeSeconds the time in seconds for each segment of the path
     * @param poseConsumer a consumer that accepts the interpolated pose at each step
     * @param interpolator the interpolator used to compute intermediate points between path elements
     * @param systems the subsystems required by this command
     */
    public Interpolate(List<T> path, double segmentTimeSeconds, Consumer<T> poseConsumer, PointInterpolator<T> interpolator, Subsystem... systems) {
        this.path = path;
        this.segmentTime = segmentTimeSeconds;
        this.poseConsumer = poseConsumer;
        this.interpolator = interpolator;

        super.addRequirements(systems);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentSegment = 0;
        segmentStartTime = 0.0;
    }

    @Override
    public void execute() {
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
    }

    @Override
    public boolean isFinished() {
        return currentSegment >= path.size() - 1;
    }

    @Override
    public void end(boolean interrupted) {
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
