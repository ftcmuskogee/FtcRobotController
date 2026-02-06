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

@Autonomous(
        name = "PedroPathing - Red Main",
        group = "PP Autonomous",
        preselectTeleOp = "MecanumTeleOp"
)
@Configurable
public class RedOffGoalPP extends OpMode {

    // ================= PEDRO =================
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
        TO_GOAL_1,
        SHOOT,
        TO_RELOAD_1,
        RELOAD_1,
        TO_GOAL_2,
        SET_TO_RELOAD_2,
        RELOAD_2,
        TO_GOAL_3,
        OFF,
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
        follower.setStartingPose(new Pose(26, 128, Math.toRadians(45)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
    @Override
    public void start() {
        state = AutoState.TO_GOAL_1;
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

            case TO_GOAL_1:
                followOnce(paths.ToGoal1, AutoState.SHOOT);
                break;

            case SHOOT:
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);

                if (elapsed > 2000 && elapsed < 2200) intakeMotor.setPower(-1);
                else if (elapsed > 2200 && elapsed < 2450) intakeMotor.setPower(0);
                else if (elapsed > 2450 && elapsed < 2650) intakeMotor.setPower(-1);
                else if (elapsed > 2650 && elapsed < 2900) intakeMotor.setPower(0);
                else if (elapsed > 2900 && elapsed < 3100) intakeMotor.setPower(-1);
                else if (elapsed > 3100) {
                    intakeMotor.setPower(0);
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

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
                followOnce(paths.ToGoal3, AutoState.OFF);
                break;

            case OFF:
                followOnce(paths.Off, AutoState.DONE);
                break;

            case DONE:
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

    // ================= PATHS =================
    public static class Paths {
        public PathChain ToGoal1, SetToReload1, Reload1,
                ToGoal2, SetToReload2, Reload2,
                ToGoal3, Off;

        public Paths(Follower follower) {

            ToGoal1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26, 128),
                            new Pose(46, 107.5)))
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();

            SetToReload1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46, 107.5),
                            new Pose(58, 90)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Reload1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(43.5, 84),
                            new Pose(24, 84)))
                    .setTangentHeadingInterpolation()
                    .build();

            ToGoal2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24, 84),
                            new Pose(48.489, 92.078)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            SetToReload2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46, 108),
                            new Pose(55.022, 83.494)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Reload2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42, 60),
                            new Pose(24, 60)))
                    .setTangentHeadingInterpolation()
                    .build();

            ToGoal3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24, 60),
                            new Pose(49.489, 87.606)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Off = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46, 107.5),
                            new Pose(52, 116.5)))
                    .setConstantHeadingInterpolation(Math.toRadians(45))
                    .build();
        }
    }
}
