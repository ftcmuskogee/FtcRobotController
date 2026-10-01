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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    double closeP = 73;
    double closeF = 14;
    double closeV = 1500;
    private DcMotor intakeMotor;
    private CRServo Gate;
    private Servo Hood;

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

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "S1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "S2");
        intakeMotor = hardwareMap.get(DcMotor.class, "NTK");
        Gate = hardwareMap.get(CRServo.class,"Gate");
        Hood = hardwareMap.get(Servo.class, "Hood");

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(closeP, 0, 0, closeF);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        paths = new Paths(follower);

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

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.addData("State - ", state);
        panelsTelemetry.addData("X - ", Math.round(follower.getPose().getX()));
        panelsTelemetry.addData("Y - ", Math.round(follower.getPose().getY()));
        panelsTelemetry.addData("Heading - ", Math.round(Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.addData("Estimated Voltage - ", Math.round(volts));
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
                startFollow(paths.toGoal1());
                if (elapsed() >= 500 && elapsed() <= 1000) {
                    Gate.setPower(1);
                } else {
                    Gate.setPower(0);
                }
                shooterMotor1.setVelocity(closeV);
                shooterMotor2.setVelocity(closeV);
                stopEarly = 0;
                if (elapsed() > 1000) transitionTo(AutoState.SHOOT1);
                break;

            case SHOOT1: //                                                                         SHOOT 1
                if (elapsed() >= 1000 && elapsed() <= 1500) {
                    Gate.setPower(-1);
                } else {
                    Gate.setPower(0);
                }
                shooterMotor1.setVelocity(closeV);
                shooterMotor2.setVelocity(closeV);
                if (!follower.isBusy() && (elapsed() >= 1850 && elapsed() <= 3250)) {
                    intakeMotor.setPower(0.75);
                } else if (elapsed() > 3250) {
                    stopMotors();
                    transitionTo(AutoState.TO_RELOAD_1);
                }
                break;

            case TO_RELOAD_1: //                                                                    TO RELOAD 1
                startFollow(paths.setToReload1());
                if (elapsed() <= 500) {
                    Gate.setPower(1);
                } else {
                    Gate.setPower(0);
                }
                stopEarly = 1;
                if (elapsed() > 500) transitionTo(AutoState.RELOAD_1);
                break;

            case RELOAD_1: //                                                                       RELOAD 1
                Gate.setPower(0);
                stopEarly = 2;
                startFollow(paths.reload1());
                intakeMotor.setPower(0.9);
                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_2);
                }
                break;

            case TO_GOAL_2: //                                                                      TO GOAL 2
                Gate.setPower(0);
                stopEarly = 0;
                shooterMotor1.setVelocity(closeV);
                shooterMotor2.setVelocity(closeV);
                followOnce(paths.toGoal2(), AutoState.SHOOT2);
                break;

            case SHOOT2: //                                                                         SHOOT 2
                if (elapsed() >= 500 && elapsed() <= 1000) {
                    Gate.setPower(-1);
                } else {
                    Gate.setPower(0);
                }
                if (!follower.isBusy() && (elapsed() >= 1250 && elapsed() <= 2650)) {
                    intakeMotor.setPower(0.75);
                } else if (elapsed() >= 2650) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_2);
                }
                break;

            case SET_TO_RELOAD_2: //                                                                TO RELOAD 2
                startFollow(paths.setToReload2());
                if (elapsed() <= 500) {
                    Gate.setPower(1);
                } else {
                    Gate.setPower(0);
                }
                stopEarly = 1;
                if (elapsed() > 500) transitionTo(AutoState.RELOAD_2);
                break;

            case RELOAD_2: //                                                                       RELOAD 2
                Gate.setPower(0);
                stopEarly = 2;
                startFollow(paths.reload2());
                intakeMotor.setPower(0.9);
                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.BackUp);
                }
                break;

            case BackUp: //                                                                         BACK UP
                Gate.setPower(0);
                stopEarly = 0;
                followOnce(paths.backUp(), AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3: //                                                                      TO GOAL 3
                Gate.setPower(0);
                stopEarly = 0;
                shooterMotor1.setVelocity(closeV);
                shooterMotor2.setVelocity(closeV);
                followOnce(paths.toGoal3(), AutoState.SHOOT3);
                break;

            case SHOOT3: //                                                                         SHOOT 3
                if (elapsed() >= 500 && elapsed() <= 1000) {
                    Gate.setPower(-1);
                } else {
                    Gate.setPower(0);
                }
                if (!follower.isBusy() && (elapsed() >= 1250 && elapsed() <= 2650)) {
                    intakeMotor.setPower(0.75);
                } else if (elapsed() >= 2650) {
                    stopMotors();
                    transitionTo(AutoState.SET_TO_RELOAD_3);
                }
                break;

            case SET_TO_RELOAD_3: //                                                                TO RELOAD 3
                startFollow(paths.setToReload3());
                if (elapsed() <= 500) {
                    Gate.setPower(1);
                } else {
                    Gate.setPower(0);
                }
                stopEarly = 1;
                if (elapsed() > 500) transitionTo(AutoState.RELOAD_3);
                break;

            case RELOAD_3: //                                                                       RELOAD 3
                Gate.setPower(0);

                stopEarly = 2;

                startFollow(paths.reload3());

                intakeMotor.setPower(0.9);

                if (follower.getPathCompletion() >= 0.95) {
                    stopMotors();
                    transitionTo(AutoState.TO_GOAL_4);
                }
                break;

            case TO_GOAL_4: //                                                                      TO GOAL 4
                Gate.setPower(0);
                stopEarly = 0;
                shooterMotor1.setVelocity(closeV);
                shooterMotor2.setVelocity(closeV);
                followOnce(paths.toGoal4(), AutoState.SHOOT4);
                break;

            case SHOOT4: //                                                                         SHOOT 4
                if (elapsed() >= 500 && elapsed() <= 1000) {
                    Gate.setPower(-1);
                } else {
                    Gate.setPower(0);
                }
                if (!follower.isBusy() && (elapsed() >= 850 && elapsed() <= 2250)) {
                    intakeMotor.setPower(0.75);
                } else if (elapsed() >= 2250) {
                    stopMotors();
                    transitionTo(AutoState.Off);
                    break;
                }
                break;

            case Off: //                                                                            OFF
                Gate.setPower(0);
                stopEarly = 3;
                closeV = 0.25;
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE: //                                                                           DONE
                Gate.setPower(0);
                stop();
                break;
        }
    }

    // ================= HELPERS =================
    private void stopMotors() {
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