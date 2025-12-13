package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Red Auto - Main", group = "Red")
public class RedAutoOffGoal extends LinearOpMode {

        // Declare OpMode members.
        private DcMotor frontLeftMotor = null;
        private DcMotor backLeftMotor = null;
        private DcMotor frontRightMotor = null;
        private DcMotor backRightMotor = null;

        private DcMotor shooterMotor = null;


        private ElapsedTime runtime = new ElapsedTime();


        static final double     FORWARD_SPEED = 0.6; // For directions from the left joystick.
        static final double     TURN_SPEED    = 0.2; // For rotations from the right joystick.

        @Override
        public void runOpMode() {

            // Initialize the drive system variables.
            frontLeftMotor = hardwareMap.dcMotor.get("LF");
            backLeftMotor = hardwareMap.dcMotor.get("LB");
            frontRightMotor = hardwareMap.dcMotor.get("RF");
            backRightMotor = hardwareMap.dcMotor.get("RB");
            DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "NTK");
            // Initialize the shooting motor and reload-kickstand servo.
            shooterMotor = hardwareMap.get(DcMotor.class, "SM");
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);


            // If no input, the robot won't drift
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");
            telemetry.update();

            // Wait for the game to start (driver presses START)
            waitForStart();

            // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

            // Move to launch position
            frontLeftMotor.setPower(-FORWARD_SPEED);
            backLeftMotor.setPower(-FORWARD_SPEED);
            frontRightMotor.setPower(-FORWARD_SPEED);
            backRightMotor.setPower(-FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.8)) {
                telemetry.addData("Path", "Moving to Launch Position: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            sleep(100);

            // -------------------------
            // 2. SHOOT FIRST TWO BALLS
            // -------------------------
            shooterMotor.setPower(0.95);
            sleep(1500); // spin-up
            intakeMotor.setPower(-1);
            sleep(200);  // feed ball
            intakeMotor.setPower(0);
            sleep(1200);  // shooter
            intakeMotor.setPower(-1);
            sleep(300); // feed ball
            intakeMotor.setPower(0);
            shooterMotor.setPower(0);
            sleep(150);


            setDrivePower(-FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
            runFor(0.8);

            setDrivePower(TURN_SPEED, TURN_SPEED, -TURN_SPEED, -TURN_SPEED);
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
            setDrivePower(-TURN_SPEED, -TURN_SPEED, TURN_SPEED, TURN_SPEED);
            runFor(0.3);
            setDrivePower(TURN_SPEED, TURN_SPEED, TURN_SPEED, TURN_SPEED);
            runFor(.3);
            stopDrive();

            // --------------------------------------------------------------------------------------
            // 7. SHOOT AGAIN
            // --------------------------------------------------------------------------------------
            shooterMotor.setPower(1);
            sleep(1500); // spin-up
            intakeMotor.setPower(-1);
            sleep(200);  // feed ball
            intakeMotor.setPower(0);
            sleep(1200);  // shooter
            intakeMotor.setPower(-1);
            sleep(300); // feed ball
            intakeMotor.setPower(0);
            shooterMotor.setPower(0);
            sleep(150);

            sleep(300);
            shooterMotor.setPower(0);

            runFor(2.5);

            // --------------------------------------------------------------------------------------
            // 8. STRAFE OFF LAUNCH LINE
            // --------------------------------------------------------------------------------------
            setDrivePower(FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED, -FORWARD_SPEED);
            runFor(0.7);

            stopDrive();

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(200);
        }

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

    // current code = 10 second auto
}



