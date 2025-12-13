package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Auto - Main !Intake!", group = "Blue")
public class BlueAutoOffGoalIntake extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor shooterMotor = null;
    private DcMotor intakeMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Hardware map
        frontLeftMotor  = hardwareMap.dcMotor.get("LF");
        backLeftMotor   = hardwareMap.dcMotor.get("LB");
        frontRightMotor = hardwareMap.dcMotor.get("RF");
        backRightMotor  = hardwareMap.dcMotor.get("RB");
        shooterMotor    = hardwareMap.get(DcMotor.class, "SM");
        intakeMotor     = hardwareMap.get(DcMotor.class, "NTK");

        // Directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when power = 0
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // --------------------------------------------------------------------------------------
        // 1. Move to launch position
        // --------------------------------------------------------------------------------------
        setDrivePower(-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
        runFor(0.8);

        stopDrive();
        sleep(100);

        // --------------------------------------------------------------------------------------
        // 2. SHOOT FIRST TWO BALLS
        // --------------------------------------------------------------------------------------
        shooterMotor.setPower(0.95);
        runFor(1.5); // spin-up
        intakeMotor.setPower(-1);
        runFor(0.2);  // feed ball
        intakeMotor.setPower(0);
        runFor(1.2);  // shooter recovery time (prevents weak shot)
        intakeMotor.setPower(-1);
        runFor(0.3); // feed ball
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
        runFor(0.15);

        // --------------------------------------------------------------------------------------
        // 3. Back Up, Turn to PPG (intake pickup)
        // --------------------------------------------------------------------------------------
        setDrivePower(-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
        runFor(0.8);

        setDrivePower(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED);
        runFor(0.18);

        stopDrive();
        sleep(100);

        // --------------------------------------------------------------------------------------
        // 4. Intake PPG? GPP?
        // --------------------------------------------------------------------------------------
        intakeMotor.setPower(-1);
        sleep(100);

        setDrivePower(FORWARD_SPEED/2, FORWARD_SPEED/2, FORWARD_SPEED/2, FORWARD_SPEED/2);
        runFor(2.40);

        stopDrive();
        intakeMotor.setPower(0);

        // --------------------------------------------------------------------------------------
        // 5. Move backward from PPG
        // --------------------------------------------------------------------------------------
        setDrivePower(-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
        runFor(0.7);

        // --------------------------------------------------------------------------------------
        // 6. Turn back to goal
        // --------------------------------------------------------------------------------------
        setDrivePower(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED);
        runFor(0.3);
        setDrivePower(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
        runFor(.3);
        stopDrive();

        // --------------------------------------------------------------------------------------
        // 7. SHOOT AGAIN
        // --------------------------------------------------------------------------------------
        shooterMotor.setPower(1);
        sleep(250);

        for (int i = 0; i < 2; i++) {
            sleep(750);
            intakeMotor.setPower(-1);
            sleep(500);
            intakeMotor.setPower(0);
        }

        sleep(300);
        shooterMotor.setPower(0);

        runFor(2.5);

        // --------------------------------------------------------------------------------------
        // 8. STRAFE OFF LAUNCH LINE
        // --------------------------------------------------------------------------------------
        setDrivePower(-FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, -FORWARD_SPEED);
        runFor(0.7);

        stopDrive();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(200);
    }

    // ==========================================================================================
    // Helper Functions
    // ==========================================================================================

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0, 0);
    }

    private void runFor(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addData("Running For", "%.2f sec", seconds);
            telemetry.update();
        }
    }
}
