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
        static final double     TURN_SPEED    = 0.5; // For rotations from the right joystick.

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
            sleep(200);  // feed bal
            intakeMotor.setPower(0);
            sleep(1200);  // shooter
            intakeMotor.setPower(-1);
            sleep(300); // feed ball
            intakeMotor.setPower(0);
            shooterMotor.setPower(0);
            sleep(150);

            // Move off of launch line (right strafe)
            frontLeftMotor.setPower(FORWARD_SPEED);
            backLeftMotor.setPower(-FORWARD_SPEED);
            frontRightMotor.setPower(-FORWARD_SPEED);
            backRightMotor.setPower(FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Moving Off of Launch Line: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            sleep(100);

            // current code = 10 second auto
        }



    }
