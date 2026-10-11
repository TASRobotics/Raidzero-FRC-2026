package frc.lib;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

@FunctionalInterface
public interface VisionMeasurementConsumer {
    /**
     * Submits a vision pose measurement
     *
     * @param pose      The estimated robot pose from vision
     * @param timestamp The measurement timestamp in seconds
     * @param stdDevs   The 3x1 standard deviation matrix (x, y, rotation)
     */
    void add(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
}
