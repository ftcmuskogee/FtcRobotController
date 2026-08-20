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

@Autonomous(name = "Back BLUE Only Preload", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueBACKPP extends OpMode {

    // ================= TELEMETRY =================
    private TelemetryManager panelsTelemetry;

    // ================= PATHING =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    double farP = 13;
    double farF = 15;
    double farV = 2041.8773;
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
        toShoot,
        SHOOT,
        Off,
        DONE
    }

    // ================= FLEXIBLE START POSE =================
    public Pose startPose = new Pose(54.75, 9, Math.toRadians(90));

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
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(farP, 0, 0, farF);
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        Hood.setPosition(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
    public void start() {
        state = AutoState.toShoot;
        lastState = null;
        stateStartTime = System.currentTimeMillis();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.addData("State - ", state);
        panelsTelemetry.addData("X - ", Math.round(follower.getPose().getX()));
        panelsTelemetry.addData("Y - ", Math.round(follower.getPose().getY()));
        panelsTelemetry.addData("Heading - ", Math.round(Math.toDegrees(follower.getPose().getHeading())));
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

            case toShoot: //                                                                        Turn To Goal
                Gate.setPower(0);
                stopEarly = 3;
                shooterMotor1.setVelocity(farV);
                shooterMotor2.setVelocity(farV);
                followOnce(paths.look2Goal(), AutoState.SHOOT);
                break;

            case SHOOT: //                                                                         SHOOT 1
                if (elapsed() >= 2000 && elapsed() <= 2750) Gate.setPower(-1); else Gate.setPower(0);
                shooterMotor1.setVelocity(farV);
                shooterMotor2.setVelocity(farV);
                if (!follower.isBusy() && (elapsed() >= 3000 && elapsed() <= 4350)) {
                    intakeMotor.setPower(-1);
                } else if (elapsed() > 4350) {
                    stopMotors();
                    transitionTo(AutoState.Off);
                }
                break;

            case Off: //                                                                            OFF
                Gate.setPower(0);
                stopEarly = 3;
                shooterMotor1.setVelocity(500);
                shooterMotor2.setVelocity(500);
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE: //                                                                           DONE
                Gate.setPower(0);
                shooterMotor1.setVelocity(0);
                shooterMotor2.setVelocity(0);
                Hood.setPosition(0);
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

        public PathChain look2Goal() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(54,15)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(109))
                    .build();
        }

        public PathChain off() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(53, 35)))
                    .setLinearHeadingInterpolation(pathStartPose().getHeading(), Math.toRadians(180))
                    .build();
        }
    }
}