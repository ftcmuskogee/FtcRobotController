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

@Autonomous(name = "Pedro Pathing - Blue Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueAutoOffGoal extends OpMode {

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
    final long INTAKE_TIME_MS = 3000;

    private enum AutoState {
        ToGoal1,
        SHOOT,
        TO_RELOAD_1,
        RELOAD_1,
        TO_GOAL_2,
        SET_TO_RELOAD_2,
        RELOAD_2,
        TO_GOAL_3,
        SET_TO_RELOAD_3,
        Off,
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
        follower.setStartingPose(new Pose(26, 128, Math.toRadians(135)));

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
                followOnce(paths.ToGoal1, AutoState.SHOOT);
                break;

            // -------- SHOOT --------
            case SHOOT:
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);

                if (elapsed > 2000 && elapsed < 2200) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2200 && elapsed < 2450) {
                    intakeMotor.setPower(0);
                } else if (elapsed > 2450 && elapsed < 2650) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2650 && elapsed < 2900) {
                    intakeMotor.setPower(0);
                } else if (elapsed > 2900 && elapsed < 3100) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 3100) {
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
                if (stateStartTime == 0) {
                    follower.followPath(paths.Reload1);
                }

                // run intake for 3 seconds
                if (stateStartTime < INTAKE_TIME_MS) {
                    intakeMotor.setPower(-1);
                } else {
                    intakeMotor.setPower(0);
                }

                // leave only when BOTH are done
                if (elapsed >= INTAKE_TIME_MS && !follower.isBusy()) {
                    transitionTo(AutoState.TO_GOAL_2);
                }
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
                followOnce(paths.ToGoal2, AutoState.Off);
                break;

            case Off:
                followOnce(paths.Off, AutoState.DONE);
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
        public PathChain ToGoal2;
        public PathChain SetToReload2;
        public PathChain Reload2;
        public PathChain ToGoal3;
        public PathChain Off;

        public Paths(Follower follower) {
            ToGoal1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.000, 128.000),
                                    new Pose(49.000, 108.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            SetToReload1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.000, 108.500),
                                    new Pose(53.900, 92.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Reload1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.500, 92.500),
                                    new Pose(25.000, 92.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToGoal2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.000, 92.500),
                                    new Pose(48.489, 108.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            SetToReload2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.489, 108.000),
                                    new Pose(55.122, 68.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            Reload2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.122, 68.000),
                                    new Pose(24.000, 68.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToGoal3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 68.000),
                                    new Pose(49.489, 108.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Off = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.489, 108.000),
                                    new Pose(53.000, 116.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();
        }
    }
}