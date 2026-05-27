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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumTeleOp;

@Autonomous(name = "Shoot -9- BLUE Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueOffGoalPP extends OpMode {

    // ================= TELEMETRY =================
    private TelemetryManager panelsTelemetry;

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private DcMotor intakeMotor;

    // ================= STATE =================
    private AutoState state;
    private AutoState lastState;
    private long stateStartTime;

    private boolean pathStarted = false;

    public int stopEarly = 0;

    private enum AutoState {
        ToGoal1,
        SHOOT1,
        TO_RELOAD_1,
        RELOAD_1,
        TO_GOAL_2,
        SHOOT2,
        SET_TO_RELOAD_2,
        RELOAD_2,
        BackUp,
        TO_GOAL_3,
        SHOOT3,
        Off,
        DONE
    }

    // ================= FLEXIBLE START POSE =================
    public Pose startPose = new Pose(26.0, 128.0, Math.toRadians(135));

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

        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
    @Override
    public void start() {
        state = AutoState.ToGoal1;
        lastState = null;
        stateStartTime = System.currentTimeMillis();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // ================= STATE TIMER =================
    private void updateStateTimer() {
        if (state != lastState) {
            stateStartTime = System.currentTimeMillis();
            lastState = state;
            pathStarted = false;
        }
    }

    private long elapsed() {
        return System.currentTimeMillis() - stateStartTime;
    }

    // ================= AUTO =================
    private void updateAuto() {

        switch (state) {

            case ToGoal1:
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
                stopEarly = 0;
                followOnce(paths.toGoal1(), AutoState.SHOOT1);
                break;

            case SHOOT1:
                if (elapsed() < 2500) {
                    intakeMotor.setPower(-1);
                } else {
                    stopMotors();
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

            case TO_RELOAD_1:
                stopEarly = 1;
                followOnce(paths.setToReload1(), AutoState.RELOAD_1);
                break;

            case RELOAD_1:

                startFollow(paths.reload1());

                stopEarly = 2;

                if (follower.getPathCompletion() >= 0.7) {
                    shooterMotor1.setPower(-0.85);
                    shooterMotor2.setPower(-0.85);
                }
                if (follower.getPathCompletion() <= 0.85) {
                    intakeMotor.setPower(-0.85);
                } else {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_2);
                }
                break;

            case TO_GOAL_2:
                stopEarly = 0;
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
                followOnce(paths.toGoal2(), AutoState.SHOOT2);
                break;

            case SHOOT2:
                if (elapsed() < 2500) {
                    intakeMotor.setPower(-1);
                } else {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_2);
                }
                break;

            case SET_TO_RELOAD_2:
                stopEarly = 1;
                followOnce(paths.setToReload2(), AutoState.RELOAD_2);
                break;

            case RELOAD_2:

                stopEarly = 2;

                startFollow(paths.reload2());

                if (follower.getPathCompletion() >= 0.7) {
                    shooterMotor1.setPower(-0.85);
                    shooterMotor2.setPower(-0.85);
                }
                if (follower.getPathCompletion() <= 0.85) {
                    intakeMotor.setPower(-0.85);
                } else {
                    stopMotors();
                    transitionTo(AutoState.BackUp);
                }
                break;

            case BackUp:
                stopEarly = 0;
                followOnce(paths.backUp(), AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3:
                stopEarly = 0;
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
                followOnce(paths.toGoal3(), AutoState.SHOOT3);
                break;

            case SHOOT3:
                if (elapsed() < 2500) {
                    intakeMotor.setPower(-1);
                } else {
                    stopMotors();
                    transitionTo(AutoState.Off);
                }
                break;

            case Off:
                stopEarly = 3;
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE:
                stopMotors();
                break;
        }
    }

    // ================= HELPERS =================
    private void stopMotors() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        intakeMotor.setPower(0);
    }

    private void transitionTo(AutoState next) {
        state = next;
    }

    private void startFollow(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }
    }
    private void followOnce(PathChain path, AutoState next) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }

        double threshold =
                stopEarly == 0 ? 0.9825 :
                        stopEarly == 1 ? 0.9875 :
                                stopEarly == 2 ? 0.9975 :
                                    0.75;

        if (follower.isBusy() && follower.getPathCompletion() >= threshold) {
            transitionTo(next);
        }

        if (!follower.isBusy()) {
            transitionTo(next);
        }
    }

    // ================= PATHS =================
    public static class Paths {

        private final Follower follower;

        public Paths(Follower follower) {
            this.follower = follower;
        }

        private Pose start() {
            return follower.getPose();
        }

        public PathChain toGoal1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(52, 102.5)))
                    .setConstantHeadingInterpolation(start().getHeading())
                    .build();
        }

        public PathChain setToReload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(53.5, 94.25)))
                    .setLinearHeadingInterpolation(start().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain reload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(22.5, 94.25)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain toGoal2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(52, 102.5)))
                    .setLinearHeadingInterpolation(start().getHeading(), Math.toRadians(139))
                    .build();
        }

        public PathChain setToReload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(55, 71)))
                    .setLinearHeadingInterpolation(start().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain reload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(15, 71)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain backUp() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(55, 71)))
                    .setConstantHeadingInterpolation(start().getHeading())
                    .build();
        }

        public PathChain toGoal3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(52, 100)))
                    .setLinearHeadingInterpolation(start().getHeading(), Math.toRadians(139))
                    .build();
        }

        public PathChain off() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start(), new Pose(53, 116.5)))
                    .setConstantHeadingInterpolation(start().getHeading())
                    .build();
        }
    }
}