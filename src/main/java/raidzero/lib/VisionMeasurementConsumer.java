package raidzero.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
