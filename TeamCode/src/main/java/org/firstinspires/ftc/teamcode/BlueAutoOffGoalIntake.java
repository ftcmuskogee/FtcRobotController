package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Auto - Main !Intake!", group = "Blue", preselectTeleOp = "MecanumTeleop")
public class BlueAutoOffGoalIntake extends LinearOpMode {

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor shooterMotor1;
    DcMotor shooterMotor2;
    DcMotor intakeMotor;
    ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        // Hardware map
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        backLeftMotor = hardwareMap.dcMotor.get("BL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        backRightMotor = hardwareMap.dcMotor.get("BR");
        shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
        intakeMotor = hardwareMap.dcMotor.get("NTK");

        // Directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when power = 0
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        shooterMotor1.setPower(1);
        shooterMotor2.setPower(1);
        runFor(3); // spin-up
        intakeMotor.setPower(-1);
        runFor(0.2);  // feed ball
        intakeMotor.setPower(0);
        runFor(1.5);  // shooter recovery time (prevents weak shot)
        intakeMotor.setPower(-1);
        runFor(0.3); // feed ball
        intakeMotor.setPower(0);
        runFor(1.5);
        intakeMotor.setPower(-1);
        runFor(0.2);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        runFor(0.15);

        // --------------------------------------------------------------------------------------
        // 3. Back Up, Turn to PPG (intake pickup)
        // --------------------------------------------------------------------------------------
        setDrivePower(-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
        runFor(0.5);

        setDrivePower(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED);
        runFor(0.30);

        stopDrive();
        sleep(100);

        // --------------------------------------------------------------------------------------
        // 4. Intake PPG? GPP?
        // --------------------------------------------------------------------------------------
        intakeMotor.setPower(-1);
        sleep(100);

        setDrivePower(FORWARD_SPEED/2, FORWARD_SPEED/2, FORWARD_SPEED/2, FORWARD_SPEED/2);
        runFor(3);

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
        runFor(0.28);
        setDrivePower(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
        runFor(.3);
        stopDrive();

        // --------------------------------------------------------------------------------------
        // 7. SHOOT AGAIN
        // --------------------------------------------------------------------------------------
        shooterMotor1.setPower(1);
        shooterMotor2.setPower(1);
        runFor(2); // spin-up
        intakeMotor.setPower(-1);
        runFor(0.2);  // feed ball
        intakeMotor.setPower(0);
        runFor(1.2);  // shooter recovery time (prevents weak shot)
        intakeMotor.setPower(-1);
        runFor(0.3); // feed ball
        intakeMotor.setPower(0);
        runFor(1.2);
        intakeMotor.setPower(-1);
        runFor(0.2);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        runFor(0.15);

        sleep(300);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);

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
