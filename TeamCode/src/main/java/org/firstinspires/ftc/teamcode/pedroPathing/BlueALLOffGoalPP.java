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
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name = "Shoot =12= BLUE Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueALLOffGoalPP extends OpMode {

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
    public final double initShootPow = 0.895;
    public double shootPow = initShootPow;
    public double shootAddPow = 0;
    public double needX = 18.25;

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
        SET_TO_RELOAD_3,
        RELOAD_3,
        TO_GOAL_4,
        SHOOT4,
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
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

    // ==================== Voltage ======================
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result == Double.POSITIVE_INFINITY ? 0 : result;
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        double volts = getBatteryVoltage();

        if (volts <= 10) {
            shootPow = 1;
        } else {
            shootPow = initShootPow + shootAddPow;
        }

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.debug("State - ",state);
        panelsTelemetry.debug("X - ",Math.round(follower.getPose().getX()));
        panelsTelemetry.debug("Y - ",Math.round(follower.getPose().getY()));
        panelsTelemetry.debug("Heading - ",Math.round(Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.debug("Shooter Power - ",Math.round(shootPow));
        panelsTelemetry.debug("Estimated Voltage - ",Math.round(volts));
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

            case ToGoal1: //                                                                        TO GOAL 1
                if (elapsed() < 150) {
                    shooterMotor1.setPower(-.75);
                    shooterMotor2.setPower(-.75);
                    intakeMotor.setPower(-0.125);
                } else {
                    shooterMotor1.setPower(shootPow);
                    shooterMotor2.setPower(shootPow);
                    intakeMotor.setPower(0);
                }
                stopEarly = 0;
                followOnce(paths.toGoal1(), AutoState.SHOOT1);
                break;

            case SHOOT1: //                                                                        SHOOT 1
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                if (!follower.isBusy() && (elapsed() >= 450 && elapsed() <= 2675)) {
                    intakeMotor.setPower(1);
                    if (shootPow + 0.01 <= 1) {
                        shootAddPow += 0.01;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

            case TO_RELOAD_1: //                                                                        To RELOAD 1
                stopEarly = 1;
                followOnce(paths.setToReload1(), AutoState.RELOAD_1);
                break;

            case RELOAD_1: //                                                                        RELOAD 1

                needX = 18.25;

                stopEarly = 2;

                startFollow(paths.reload1());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.35) {
                    shooterMotor1.setPower(-0.85);
                    shooterMotor2.setPower(-0.85);
                }
                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_2);
                }
                break;

            case TO_GOAL_2: //                                                                        TO GOAL 2
                stopEarly = 0;
                if (elapsed() < 150) {
                    shooterMotor1.setPower(-0.75);
                    shooterMotor2.setPower(-0.75);
                    intakeMotor.setPower(-0.125);
                } else {
                    shooterMotor1.setPower(shootPow);
                    shooterMotor2.setPower(shootPow);
                    intakeMotor.setPower(0);
                }
                followOnce(paths.toGoal2(), AutoState.SHOOT2);
                break;

            case SHOOT2: //                                                                        SHOOT 2
                if (!follower.isBusy() && (elapsed() >= 500 && elapsed() <= 2675)) {
                    intakeMotor.setPower(1);
                    if (shootPow + 0.01 <= 1) {
                        shootAddPow += 0.01;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_2);
                }
                break;

            case SET_TO_RELOAD_2: //                                                                        TO RELOAD 2
                stopEarly = 1;
                followOnce(paths.setToReload2(), AutoState.RELOAD_2);
                break;

            case RELOAD_2: //                                                                        RELOAD 2

                needX = 12;

                stopEarly = 2;

                startFollow(paths.reload2());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.35) {
                    shooterMotor1.setPower(-0.85);
                    shooterMotor2.setPower(-0.85);
                }
                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.BackUp);
                }
                break;

            case BackUp: //                                                                        BACK UP
                stopEarly = 0;
                if (elapsed() < 250) {
                    shooterMotor1.setPower(-0.75);
                    shooterMotor2.setPower(-0.75);
                    intakeMotor.setPower(-0.125);
                } else {
                    intakeMotor.setPower(0);
                }
                followOnce(paths.backUp(), AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3: //                                                                        TO GOAL 3
                stopEarly = 0;
                if (elapsed() < 150) {
                    shooterMotor1.setPower(-0.75);
                    shooterMotor2.setPower(-0.75);
                } else {
                    shooterMotor1.setPower(shootPow);
                    shooterMotor2.setPower(shootPow);
                }
                followOnce(paths.toGoal3(), AutoState.SHOOT3);
                break;

            case SHOOT3: //                                                                        SHOOT 3
                if (!follower.isBusy() && (elapsed() >= 500 && elapsed() <= 2675)) {
                    intakeMotor.setPower(1);
                    if (shootPow + 0.01 <= 1) {
                        shootAddPow += 0.01;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_3);
                }
                break;

            case SET_TO_RELOAD_3: //                                                                        TO RELOAD 3
                stopEarly = 1;
                followOnce(paths.setToReload3(), AutoState.RELOAD_3);
                break;

            case RELOAD_3: //                                                                        RELOAD 3

                needX = 12;

                stopEarly = 2;

                startFollow(paths.reload3());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.35) {
                    shooterMotor1.setPower(-0.85);
                    shooterMotor2.setPower(-0.85);
                }
                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_4);
                }
                break;

            case TO_GOAL_4: //                                                                        TO GOAL 4
                stopEarly = 0;
                if (elapsed() < 250) {
                    shooterMotor1.setPower(-0.75);
                    shooterMotor2.setPower(-0.75);
                    intakeMotor.setPower(-0.125);
                } else {
                    shooterMotor1.setPower(shootPow);
                    shooterMotor2.setPower(shootPow);
                    intakeMotor.setPower(0);
                }
                followOnce(paths.toGoal4(), AutoState.SHOOT4);
                break;

            case SHOOT4: //                                                                        SHOOT 4
                if (!follower.isBusy() && (elapsed() >= 500 && elapsed() <= 2675)) {
                    intakeMotor.setPower(1);
                    if (shootPow + 0.015 <= 1) {
                        shootAddPow += 0.015;
                    } else {
                        shootPow = 1;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.Off);
                    break;
                }
                break;

            case Off: //                                                                        OFF
                stopEarly = 3;
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE: //                                                                        DONE
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
        shootAddPow = 0;
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
                stopEarly == 0 ? 1.5 :
                        stopEarly == 1 ? 0.75 :
                                stopEarly == 2 ? 0.25 :
                                    2.5;

        if (follower.isRobotStuck()) {
            transitionTo(next);
        } else if (follower.isBusy() && follower.getDistanceRemaining() <= threshold) {
            transitionTo(next);
        } else if (!follower.isBusy()) {
            transitionTo(next);
        }
    }

    // ================= PATHS =================
    public static class Paths {

        private final Follower follower;

        public Paths(Follower follower) {
            this.follower = follower;
        }

        private Pose pathStartPose() {
            return follower.getPose();
        }

        public PathChain toGoal1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(52, 102.5)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }

        public PathChain setToReload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(53.5, 95.5)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain reload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(21, 95.5)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain toGoal2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(52, 102.5)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(139))
                    .build();
        }

        public PathChain setToReload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(55, 72)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain reload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(18.75, 72)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain backUp() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(55, 72)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain toGoal3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(52, 102.5)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(139))
                    .build();
        }

        public PathChain setToReload3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(55, 48.75)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }

        public PathChain reload3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(18.75, 48.75)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain toGoal4() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(52, 102.5)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(139))
                    .build();
        }

        public PathChain off() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(53, 117)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }
    }
}