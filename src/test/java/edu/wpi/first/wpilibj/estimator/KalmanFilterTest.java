package edu.wpi.first.wpilibj.estimator;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulatorTest;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N6;
import org.ejml.simple.SimpleMatrix;
import org.junit.Before;
import org.junit.Test;

import java.util.List;
import java.util.Random;

import static edu.wpi.first.wpilibj.controller.LinearSystemLoopTest.kDt;

public class KalmanFilterTest {

    static {
        LinearQuadraticRegulatorTest.createArm();
        LinearQuadraticRegulatorTest.createElevator();
    }

    @Test
    public void testElevatorKalmanFilter() {
        var plant = LinearQuadraticRegulatorTest.elevatorPlant;

        var Q = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.05, 1.0);
        var R = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0001);

        var filter = new KalmanFilter<>(Nat.N2(), Nat.N1(), Nat.N1(), plant, Q, R, kDt);

        var p = filter.getP();
        var gain = filter.getXhat();

        System.out.printf("p: \n%s\n: gain: \n%s\n", p, gain);
    }

    @Test
    public void testLocalization() {

        var offset = new Random();
        var trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(), new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
            new TrajectoryConfig(0.5, 2)
        );
        var random = new Random();

        /*
        So our system has 6 states: [x, vx, y, vy, theta, omega]
        Our inputs are vx, vy, and omega
        Our outputs are x, y and theta


        [vx, ax, vy, ay, omega, alpha]^T = A * [[x, vx, y, vy, theta, omega]^T + B * [vx, vy, omega]^T
        and [x, y, theta] = C * [x, vx, y, vy, theta, omega]^T + 0

         */

        var system = new LinearSystem<>(Nat.N6(), Nat.N3(), Nat.N3(),
            new MatBuilder<>(Nat.N6(), Nat.N6()).fill( // A
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0),
            new MatBuilder<>(Nat.N6(), Nat.N3()).fill( // B
                0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            ),
            new MatBuilder<>(Nat.N3(), Nat.N6()).fill( // C
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0
            ),
            new Matrix<>(new SimpleMatrix(3, 3)), // D
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(-4, -4, -12),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(4, 4, 12));

        var filter = new KalmanFilter<>(Nat.N6(), Nat.N3(), Nat.N3(),
            system,
            new MatBuilder<>(Nat.N6(), Nat.N1()).fill( 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill( 1, 1, 2 ),
            0.020
        );

        var robotPose = trajectory.getInitialPose();
        var lastControlInput = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0);
        var isDone = false;
        var lastTime = 0;
        System.out.println("refX, refY, x, y");
        while(!isDone) {
            var state = trajectory.sample(lastTime);
            lastTime += 0.020;

            var robotPoseWithNoise = robotPose.plus(new Transform2d(
                new Translation2d(
                    random.nextDouble() * (random.nextBoolean() ? 1 : -1),
                    random.nextDouble() * (random.nextBoolean() ? 1 : -1)
                ), new Rotation2d(random.nextDouble() * (random.nextBoolean() ? 2 : -2))
            ));

            filter.correct(
                lastControlInput,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                    robotPoseWithNoise.getTranslation().getX(),
                    robotPoseWithNoise.getTranslation().getY(),
                    robotPoseWithNoise.getRotation().getRadians()
                )
            );


            lastControlInput = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                state.poseMeters.getRotation().getCos() * state.velocityMetersPerSecond,
                state.poseMeters.getRotation().getSin() * state.velocityMetersPerSecond,
                state.velocityMetersPerSecond * state.curvatureRadPerMeter
            );

            System.out.println(String.format("%s, %s, %s, %s", state.poseMeters.getTranslation().getX(),
                state.poseMeters.getTranslation().getY(), filter.getXhat(0), filter.getXhat(1)));

            filter.predict(lastControlInput, 0.020);

            robotPose = state.poseMeters;
        }



    }

}
