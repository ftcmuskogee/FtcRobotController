package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Pedro Pathing - Back Blue Autonomous", group = "PP Autonomous", preselectTeleOp = "MecanumTeleop")
@Configurable
public class BlueOffBackAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private Paths paths;

    private long stateStartTime;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 0;
        stateStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ==================================================
    // ================= STATE MACHINE ==================
    // ==================================================

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Go to launch line
                follower.followPath(paths.ToLaunchLineTurntoGoal);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis();
                    pathState++;
                }
                break;

            case 2:
                // Simulated shooting wait
                if (System.currentTimeMillis() - stateStartTime > 1000) {
                    follower.followPath(paths.SettoReload);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReloadAddreloadingcodeforintake);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.BacktoLaunchLine);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    stateStartTime = System.currentTimeMillis();
                    pathState++;
                }
                break;

            case 6:
                // Second shooting wait
                if (System.currentTimeMillis() - stateStartTime > 1000) {
                    follower.followPath(paths.LeaveLaunchLine);
                    pathState++;
                }
                break;

            case 7:
                // Done
                break;
        }
    }

    // ==================================================
    // ================= PATH DEFINITIONS ===============
    // ==================================================

    public static class Paths {

        public PathChain ToLaunchLineTurntoGoal;
        public PathChain SettoReload;
        public PathChain ReloadAddreloadingcodeforintake;
        public PathChain BacktoLaunchLine;
        public PathChain LeaveLaunchLine;

        public Paths(Follower follower) {

            ToLaunchLineTurntoGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(55.000, 8.000),
                                    new Pose(55.000, 80.000) // Supposed to be 88
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(135)
                    )
                    .build();

            SettoReload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(55.000, 80.000), // Supposed to be 88
                                    new Pose(42.500, 67.500) // Supposed to be 83.75
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180)
                    )
                    .build();

            ReloadAddreloadingcodeforintake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.500, 67.500), // Supposed to be 83.75
                                    new Pose(23.500, 67.500) // Supposed to be 83.75
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();

            BacktoLaunchLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(23.500, 67.500), // Supposed to be 83.75
                                    new Pose(55.000, 80.000) // Supposed to be 88
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135)
                    )
                    .build();

            LeaveLaunchLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(55.000, 80.000), // Supposed to be 88
                                    new Pose(20.000, 85.000) // Supposed to be 93
                            )
                    )
                    .setConstantHeadingInterpolation(
                            Math.toRadians(135)
                    )
                    .build();
        }
    }
}
