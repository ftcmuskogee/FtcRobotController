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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name = "Shoot =12= RED Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class RedALLOffGoalPP extends OpMode {

    // ================= TELEMETRY =================
    private TelemetryManager panelsTelemetry;

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private DcMotor intakeMotor;
    private Servo Gate;
    private Servo Hood;

    // ================= STATE =================
    private AutoState state;
    private AutoState lastState;
    private long stateStartTime;
    private boolean pathStarted = false;
    public int stopEarly = 0;
    public final double initShootPow = 0.895;
    public double shootPow = initShootPow;
    public double shootAddPow = 0;

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
    public Pose startPose = new Pose(118.0, 128.0, Math.toRadians(45));

    // ================= INIT =================
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
        intakeMotor = hardwareMap.get(DcMotor.class, "NTK");
        Gate = hardwareMap.get(Servo.class,"Gate");
        Hood = hardwareMap.get(Servo.class, "Hood");

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        Gate.setPosition(1);
        Hood.setPosition(0.4);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
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

        if (volts <= 9.75) {
            shootPow = 1;
        } else {
            shootPow = initShootPow + shootAddPow;
        }

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.addData("State - ", state);
        panelsTelemetry.addData("X - ", Math.round(follower.getPose().getX()));
        panelsTelemetry.addData("Y - ", Math.round(follower.getPose().getY()));
        panelsTelemetry.addData("Heading - ", Math.round(Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.addData("Shooter Power - ", Math.round(shootPow));
        panelsTelemetry.addData("Estimated Voltage - ", Math.round(volts));
        panelsTelemetry.addData("Gate Status - ", (Gate.getPosition() == 0 ? "OPEN" : "CLOSED"));
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
                Gate.setPosition(.65);
                shooterMotor1.setPower(1);
                shooterMotor2.setPower(1);
                stopEarly = 0;
                followOnce(paths.toGoal1(), AutoState.SHOOT1);
                break;

            case SHOOT1: //                                                                         SHOOT 1
                if (elapsed() >= 1000) Gate.setPosition(0); else Gate.setPosition(.65);
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                if (!follower.isBusy() && (elapsed() >= 1850 && elapsed() <= 3250)) {
                    intakeMotor.setPower(0.75);
                    if (shootPow + 0.01 <= 0.95) {
                        shootAddPow += 0.01;
                    } else {
                        shootPow = 0.95;
                    }
                } else if (elapsed() > 3250) {
                    stopMotors();
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

            case TO_RELOAD_1: //                                                                    TO RELOAD 1
                Gate.setPosition(.65);
                stopEarly = 1;
                followOnce(paths.setToReload1(), AutoState.RELOAD_1);
                break;

            case RELOAD_1: //                                                                       RELOAD 1
                Gate.setPosition(.65);

                stopEarly = 2;

                startFollow(paths.reload1());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_2);
                }
                break;

            case TO_GOAL_2: //                                                                      TO GOAL 2
                Gate.setPosition(.65);
                stopEarly = 0;
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                followOnce(paths.toGoal2(), AutoState.SHOOT2);
                break;

            case SHOOT2: //                                                                         SHOOT 2
                if (elapsed() >= 50) Gate.setPosition(0); else Gate.setPosition(.65);
                if (!follower.isBusy() && (elapsed() >= 850 && elapsed() <= 2250)) {
                    intakeMotor.setPower(0.75);
                    if (shootPow + 0.005 <= 0.9) {
                        shootAddPow += 0.01;
                    } else {
                        shootPow = 0.9;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_2);
                }
                break;

            case SET_TO_RELOAD_2: //                                                                TO RELOAD 2
                Gate.setPosition(.65);
                stopEarly = 1;
                followOnce(paths.setToReload2(), AutoState.RELOAD_2);
                break;

            case RELOAD_2: //                                                                       RELOAD 2
                Gate.setPosition(.65);

                stopEarly = 2;

                startFollow(paths.reload2());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.BackUp);
                }
                break;

            case BackUp: //                                                                         BACK UP
                Gate.setPosition(.65);
                stopEarly = 0;
                followOnce(paths.backUp(), AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3: //                                                                      TO GOAL 3
                Gate.setPosition(.65);
                stopEarly = 0;
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                followOnce(paths.toGoal3(), AutoState.SHOOT3);
                break;

            case SHOOT3: //                                                                         SHOOT 3
                if (elapsed() >= 100) Gate.setPosition(0); else Gate.setPosition(.65);
                if (!follower.isBusy() && (elapsed() >= 850 && elapsed() <= 2250)) {
                    intakeMotor.setPower(0.75);
                    if (shootPow + 0.005 <= 0.9) {
                        shootAddPow += 0.01;
                    } else {
                        shootPow = 0.9;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_3);
                }
                break;

            case SET_TO_RELOAD_3: //                                                                TO RELOAD 3
                Gate.setPosition(.65);
                stopEarly = 1;
                followOnce(paths.setToReload3(), AutoState.RELOAD_3);
                break;

            case RELOAD_3: //                                                                       RELOAD 3
                Gate.setPosition(.65);

                stopEarly = 2;

                startFollow(paths.reload3());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_4);
                }
                break;

            case TO_GOAL_4: //                                                                      TO GOAL 4
                Gate.setPosition(.65);
                stopEarly = 0;
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                followOnce(paths.toGoal4(), AutoState.SHOOT4);
                break;

            case SHOOT4: //                                                                         SHOOT 4
                if (elapsed() >= 50) Gate.setPosition(0); else Gate.setPosition(.65);
                if (!follower.isBusy() && (elapsed() >= 850 && elapsed() <= 2250)) {
                    intakeMotor.setPower(0.75);
                    if (shootPow + 0.005 <= 0.9) {
                        shootAddPow += 0.015;
                    } else {
                        shootPow = 0.95;
                    }
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.Off);
                    break;
                }
                break;

            case Off: //                                                                            OFF
                Gate.setPosition(.65);
                stopEarly = 3;
                shootPow = 0.25;
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE: //                                                                           DONE
                Gate.setPosition(.65);
                stop();
                break;
        }
    }

    // ================= HELPERS =================
    private void stopMotors() {
        shootPow = 0.8;
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

        double inchesOff =
                stopEarly == 0 ? 2 :
                        stopEarly == 1 ? 1 :
                                stopEarly == 2 ? 0.5 :
                                        3;

        if (follower.isRobotStuck()) {
            transitionTo(next);
        } else if (follower.isBusy() && follower.getDistanceRemaining() <= inchesOff) {
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
                    .addPath(new BezierLine(pathStartPose(), new Pose(91, 108)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }

        public PathChain setToReload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(90, 96.5)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(0))
                    .build();
        }

        public PathChain reload1() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(125, 96.5)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain toGoal2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(95, 108)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(35))
                    .build();
        }

        public PathChain setToReload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(89, 75)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(0))
                    .build();
        }

        public PathChain reload2() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(130, 75)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain backUp() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(89, 75)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }

        public PathChain toGoal3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(95, 108)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(35))
                    .build();
        }

        public PathChain setToReload3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(90, 53.75)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(0))
                    .build();
        }

        public PathChain reload3() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(130, 53.75)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        public PathChain toGoal4() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(95, 108)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(49))
                    .build();
        }

        public PathChain off() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(90, 114)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }
    }
}