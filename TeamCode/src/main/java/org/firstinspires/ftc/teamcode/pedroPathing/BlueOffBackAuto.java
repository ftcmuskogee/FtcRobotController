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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Pedro Pathing - Back Blue Autonomous", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueOffBackAuto extends OpMode {

    // ================= FTC / PEDRO =================
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private DcMotor intakeMotor;

    // ================= STATE MACHINE =================
    private AutoState state;
    private long stateStartTime;

    private enum AutoState {
        ToGoal1,
        PRELOAD_SHOOT,
        TO_RELOAD_1,
        RELOAD_1,
        TO_GOAL_2,
        SET_TO_RELOAD_2,
        RELOAD_2,
        TO_GOAL_3,
        SET_TO_RELOAD_3,
        DONE
    }

    // ================= INIT =================
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
        intakeMotor = hardwareMap.get(DcMotor.class, "NTK");

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
    @Override
    public void start() {
        state = AutoState.ToGoal1;
        stateStartTime = System.currentTimeMillis();
    }

    // ================= LOOP =================
    @Override
    public void loop() {
        follower.update();
        updateAuto();

        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // ================= AUTO LOGIC =================
    private void updateAuto() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (state) {

            // -------- TO LAUNCH LINE --------
            case ToGoal1:
                followOnce(paths.ToGoal1, AutoState.PRELOAD_SHOOT);
            break;

            // -------- PRELOAD SHOOT --------
            case PRELOAD_SHOOT:
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);

                if (elapsed > 1000 && elapsed < 1200) intakeMotor.setPower(-1);
                else if (elapsed > 1200 && elapsed < 1450) intakeMotor.setPower(0);
                else if (elapsed > 1450 && elapsed < 1650) intakeMotor.setPower(-1);
                else if (elapsed > 1650 && elapsed < 1900) intakeMotor.setPower(0);
                else if (elapsed > 1900 && elapsed < 2100) intakeMotor.setPower(-1);
                else if (elapsed > 2100) {
                    intakeMotor.setPower(0);
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

            // -------- PATH STATES --------
            case TO_RELOAD_1:
                followOnce(paths.SetToReload1, AutoState.RELOAD_1);
                break;

            case RELOAD_1:
                followOnce(paths.Reload1, AutoState.TO_GOAL_2);
                break;

            case TO_GOAL_2:
                followOnce(paths.ToGoal2, AutoState.SET_TO_RELOAD_2);
                break;

            case SET_TO_RELOAD_2:
                followOnce(paths.SetToReload2, AutoState.RELOAD_2);
                break;

            case RELOAD_2:
                followOnce(paths.Reload2, AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3:
                followOnce(paths.ToGoal2, AutoState.SET_TO_RELOAD_3);
                break;

            case SET_TO_RELOAD_3:
                followOnce(paths.SetToReload3, AutoState.DONE);
                break;

            case DONE:
                // Autonomous complete
                break;
        }
    }

    // ================= HELPERS =================
    private void transitionTo(AutoState next) {
        state = next;
        stateStartTime = System.currentTimeMillis();
    }

    private void followOnce(PathChain path, AutoState next) {
        if (!follower.isBusy()) {
            follower.followPath(path);
            transitionTo(next);
        }
    }

    // ================= PATH DEFINITIONS =================
    public static class Paths {
        public PathChain ToGoal1;
        public PathChain SetToReload1;
        public PathChain Reload1;
        public PathChain SetToShoot1;
        public PathChain SetToReload2;
        public PathChain Reload2;
        public PathChain ToGoal2;
        public PathChain SetToReload3;

        public Paths(Follower follower) {

            ToGoal1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(55.000, 8.000),
                            new Pose(55.000, 80.000),
                            new Pose(46.000, 105.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(135)
                    )
                    .build();

            SetToReload1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 105.000),
                            new Pose(46.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180)
                    )
                    .build();

            Reload1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 84.000),
                            new Pose(24.000, 84.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            SetToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.000, 84.000),
                            new Pose(46.000, 105.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135)
                    )
                    .build();

            SetToReload2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 105.000),
                            new Pose(46.000, 59.500)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180)
                    )
                    .build();

            Reload2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.000, 59.500),
                            new Pose(24.000, 59.500)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ToGoal2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.000, 59.500),
                            new Pose(46.000, 105.000)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(135)
                    )
                    .build();

            SetToReload3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(46.000, 105.000),
                            new Pose(60.000, 60.000),
                            new Pose(58.489, 35.444)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(180)
                    )
                    .build();
        }
    }
}