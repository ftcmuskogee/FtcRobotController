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

@Autonomous(name = "Pedro Pathing - Blue Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueOffGoalPP extends OpMode {

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
    final long INTAKE_TIME_MS = 1850;

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
        follower.setMaxPowerScaling(0.9);
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
                shooterMotor1.setPower(.97);
                shooterMotor2.setPower(.97);
                followOnce(paths.ToGoal1, AutoState.SHOOT1);
                break;

            // -------- SHOOT PRELOAD --------
            case SHOOT1:

                if (elapsed < 1900) {
                    intakeMotor.setPower(-1);
                } else if (elapsed >= 1900 && elapsed < 2500) {
                    intakeMotor.setPower(-1);
                } else if (elapsed >= 2500) {
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
                if (!follower.isBusy()) {
                    follower.setMaxPowerScaling(0.3);
                    follower.followPath(paths.Reload1);
                }

                if (elapsed < INTAKE_TIME_MS) {
                    intakeMotor.setPower(-.85);
                } else {
                    intakeMotor.setPower(0);
                }

                if (elapsed >= INTAKE_TIME_MS + 250 && elapsed < INTAKE_TIME_MS + 800) {
                    shooterMotor1.setPower(-.95);
                    shooterMotor2.setPower(-.95);
                    intakeMotor.setPower(.85);
                } else if (elapsed >= INTAKE_TIME_MS + 800) {
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.TO_GOAL_2);
                }
                break;


            case TO_GOAL_2:
                follower.setMaxPowerScaling(0.9);
                shooterMotor1.setPower(.95);
                shooterMotor2.setPower(.95);
                followOnce(paths.ToGoal2, AutoState.SHOOT2);
                break;

            case SHOOT2:

                if (elapsed < 2200) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2200 && elapsed < 2500) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2500) {
                    intakeMotor.setPower(0);
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.SET_TO_RELOAD_2);
                }
                break;

            case SET_TO_RELOAD_2:
                followOnce(paths.SetToReload2,AutoState.RELOAD_2);
                break;

            case RELOAD_2:
                if (!follower.isBusy()) {
                    follower.setMaxPowerScaling(0.3);
                    follower.followPath(paths.Reload2);
                }

                if (elapsed < INTAKE_TIME_MS) {
                    intakeMotor.setPower(-.85);
                } else {
                    intakeMotor.setPower(0);
                }

                if (elapsed >= INTAKE_TIME_MS + 250 && elapsed < INTAKE_TIME_MS + 800) {
                    shooterMotor1.setPower(-.95);
                    shooterMotor2.setPower(-.95);
                    intakeMotor.setPower(.85);
                } else if (elapsed >= INTAKE_TIME_MS + 800) {
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.BackUp);
                }
                break;

            case BackUp:
                followOnce(paths.BackUp, AutoState.TO_GOAL_3);
                break;

            case TO_GOAL_3:
                shooterMotor1.setPower(.95);
                shooterMotor2.setPower(.95);
                followOnce(paths.ToGoal3, AutoState.SHOOT3);
                break;

            case SHOOT3:

                if (elapsed < 2200) {
                    follower.setMaxPowerScaling(0.9);
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2200 && elapsed < 2500) {
                    intakeMotor.setPower(-1);
                } else if (elapsed > 2500) {
                    intakeMotor.setPower(0);
                    shooterMotor1.setPower(0);
                    shooterMotor2.setPower(0);
                    transitionTo(AutoState.Off);
                }
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

    private boolean pathStarted = false;

    private void followOnce(PathChain path, AutoState next) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        } else if (!follower.isBusy()) {
            pathStarted = false;
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
        public PathChain BackUp;
        public PathChain ToGoal3;
        public PathChain Off;

        public Paths(Follower follower) {
            ToGoal1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.000, 128.000),
                                    new Pose(52.000, 102.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(139))

                    .build();

            SetToReload1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 102.500),
                                    new Pose(53.500, 94.250)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))

                    .build();

            Reload1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.500, 94.250),
                                    new Pose(22.500, 94.250)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ToGoal2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.500, 94.250),
                                    new Pose(52.000, 102.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))

                    .build();

            SetToReload2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 102.500),
                                    new Pose(55.000, 71.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))

                    .build();

            Reload2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.000, 71.000),
                                    new Pose(15.000, 71.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            BackUp = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 71.000),
                                    new Pose(55.000, 71.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            ToGoal3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.000, 71.000),
                                    new Pose(52.000, 100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))

                    .build();

            Off = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.000, 102.500),
                                    new Pose(53.000, 116.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();
        }
    }
}