package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Pedro Pathing - All Back Red Autonomous", group = "PP Autonomous", preselectTeleOp = "MecanumTeleop")
@Configurable
@Disabled
public class RedStShOffBackAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private Paths paths;

    private long stateStartTime;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(81, 8, Math.toRadians(90)));

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
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoShoot1);
                    pathState++;
                }
                break;

            case 1:
                if (System.currentTimeMillis() - stateStartTime >= paths.Shoot1){
                    pathState++;
                stateStartTime = System.currentTimeMillis();
            }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoReload1);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reload1);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoShoot2);
                    pathState++;
                }
                break;

            case 5:
                if (System.currentTimeMillis() - stateStartTime >= paths.Shoot2) {
                    stateStartTime = System.currentTimeMillis();
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoReload2);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reload2);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoShoot3);
                    pathState++;
                }
                break;

            case 9:
                if (System.currentTimeMillis() - stateStartTime >= paths.Shoot3) {
                    stateStartTime = System.currentTimeMillis();
                    pathState++;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Ahfafa);
                    pathState++;
                }
                break;

            case 11:
                break;

        }
    }

    // ==================================================
    // ================= PATH DEFINITIONS ===============
    // ==================================================

    public static class Paths {

        public PathChain SettoShoot1;
        public double Shoot1;
        public PathChain SettoReload1;
        public PathChain Reload1;
        public PathChain SettoShoot2;
        public double Shoot2;
        public PathChain SettoReload2;
        public PathChain Reload2;
        public PathChain SettoShoot3;
        public double Shoot3;
        public PathChain Ahfafa;

        public Paths(Follower follower) {
            SettoShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.000, 8.000), new Pose(81.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63.5))
                    .build();

            Shoot1 = 2000;

            SettoReload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.000, 15.000), new Pose(94.000, 31.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(63.5),Math.toRadians(0))
                    .build();

            Reload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94.000, 31.000), new Pose(130.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SettoShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.000, 35.000), new Pose(81.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63.5))
                    .build();

            Shoot2 = 2000;

            SettoReload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.000, 15.000), new Pose(98.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(63.5), Math.toRadians(0))
                    .build();

            Reload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98.000, 8.000), new Pose(135.200, 8.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SettoShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135.200, 8.000), new Pose(81.000, 15.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63.5))
                    .build();

            Shoot3 = 2000;

            Ahfafa = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.000, 15.000), new Pose(81.000, 45.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
        }