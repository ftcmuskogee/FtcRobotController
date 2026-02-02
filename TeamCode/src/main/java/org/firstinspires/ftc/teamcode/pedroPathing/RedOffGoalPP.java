package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroPathing - Red Off Goal", group = "PP Autonomous", preselectTeleOp = "MecanumTeleop")
@Configurable // Panels
public class RedOffGoalPP extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private long stateStartTime;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120, 127, Math.toRadians(43)));

        paths = new Paths(follower); // Build paths

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
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SettoShootPreload1);
                    pathState++;
                }
                break;

            case 1:
                if (System.currentTimeMillis() - stateStartTime >= paths.ShootPreload1){
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

    public static class Paths {

        public PathChain SettoShootPreload1;
        public double ShootPreload1;
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
            SettoShootPreload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.000, 127.000), new Pose(96.000, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))
                    .build();

            ShootPreload1 = 2000;

            SettoReload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 104.000), new Pose(96.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                    .build();

            Reload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 84.000), new Pose(122.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SettoShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.000, 84.000), new Pose(96.000, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                    .build();

            Shoot2 = 2000;

            SettoReload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 104.000), new Pose(96.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                    .build();

            Reload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 60.000), new Pose(122.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SettoShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.000, 60.000), new Pose(96.000, 104.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                    .build();

            Shoot3 = 2000;

            Ahfafa = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 104.000), new Pose(90.000, 110.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(43))
                    .build();
        }
    }
}
