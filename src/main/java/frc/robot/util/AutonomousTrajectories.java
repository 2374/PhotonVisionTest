package frc.robot.util;

import edu.wpi.first.math.util.Units;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.control.TrajectoryConstraint;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;

public class AutonomousTrajectories {

    public static final double SAMPLE_DISTANCE = Units.inchesToMeters(0.1);

    private final Trajectory oneMeterF;
    private final Trajectory oneMeterB;
    private final Trajectory figureEight;
    private final Trajectory fourPointSevenMeterB;
    private final Trajectory fourPointNineFiveMeterF;
    private final Trajectory sideOneMeter;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) {
        oneMeterF = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.fromDegrees(0.0))
                        .lineTo(new Vector2(1, 0.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
        oneMeterB = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.fromDegrees(0.0))
                        .lineTo(new Vector2(1, 0.0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
        figureEight = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0.0))
                        .arcTo(new Vector2(1, 1), new Vector2(0, 1), Rotation2.fromDegrees(90))
                        .arcTo(new Vector2(2, 2), new Vector2(2, 1), Rotation2.fromDegrees(180))
                        .arcTo(new Vector2(3, 1), new Vector2(2, 1), Rotation2.fromDegrees(270))
                        .arcTo(new Vector2(2, 0), new Vector2(2, 1), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(1, 1), new Vector2(2, 1), Rotation2.fromDegrees(90))
                        .arcTo(new Vector2(0, 2), new Vector2(0, 1), Rotation2.fromDegrees(180))
                        .arcTo(new Vector2(-1, 1), new Vector2(0, 1), Rotation2.fromDegrees(270))
                        .arcTo(new Vector2(0, 0), new Vector2(0, 1), Rotation2.fromDegrees(0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
        fourPointSevenMeterB = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(4.7, 0), Rotation2.fromDegrees(0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
        fourPointNineFiveMeterF = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(-4.95, 0), Rotation2.fromDegrees(0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
        sideOneMeter = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(0, -2), Rotation2.fromDegrees(0)).build(),
                trajectoryConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getOneMeterF() {
        return oneMeterF;
    }

    public Trajectory getOneMeterB() {
        return oneMeterB;
    }

    public Trajectory getFigureEight() {
        return figureEight;
    }

    public Trajectory getFourPointSevenMeterB() {
        return fourPointSevenMeterB;
    }

    public Trajectory getFourPointNineFiveMeterF() {
        return fourPointNineFiveMeterF;
    }

    public Trajectory getSideOneMeter() {
        return sideOneMeter;
    }
}
