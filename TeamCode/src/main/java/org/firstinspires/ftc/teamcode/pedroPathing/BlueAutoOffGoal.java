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

@Autonomous(name = "Pedropathing - Blue Off Goal", group = "Pedropathing")
@Configurable // Panels

public class BlueAutoOffGoal extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24.4, 125, Math.toRadians(131)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ShootPreload;
        public PathChain GetReload1;
        public PathChain Reload1;
        public PathChain ShootReload1;
        public PathChain GetReload2;
        public PathChain Reload2;
        public PathChain ShootReload2;
        public PathChain OffShootingLine;

        public Paths(Follower follower) {
            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.400, 125.000), new Pose(55.084, 94.187))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(131))
                    .build();

            GetReload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.084, 94.187), new Pose(50.323, 75))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                    .build();

            Reload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.323, 75), new Pose(15, 75))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootReload1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15, 75), new Pose(55.084, 94.357))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                    .build();

            GetReload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.084, 94.357), new Pose(51.344, 45))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                    .build();

            Reload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(51.344, 45), new Pose(15, 45))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootReload2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15, 55), new Pose(55.254, 94.697))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                    .build();

            OffShootingLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.254, 94.697), new Pose(55.254, 115.608))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Shoot preload
                follower.followPath(paths.ShootPreload);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GetReload1);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reload1);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootReload1);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GetReload2);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Reload2);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootReload2);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.OffShootingLine);
                    pathState++;
                }
                break;

            case 8:
                break;
        }

        return pathState;
    }

}

