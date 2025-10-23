/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Autonomous - Just gotta MOVE!", group="Robot")
// @Disabled
public class BlueAutoWShootable extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor shooterMotor = null;
    private Servo shooterReloader = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        // Initialize the shooting motor and reload-kickstand servo.
        shooterMotor = hardwareMap.get(DcMotor.class, "Shooter");
        shooterReloader = hardwareMap.get(Servo.class, "Reloader");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setDirection(DcMotor.Direction.FORWARD); // untested
        shooterReloader.setDirection(Servo.Direction.FORWARD); // untested

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Turn towards goal (untested)
        frontLeftDrive.setPower(-TURN_SPEED);
        backLeftDrive.setPower(-TURN_SPEED);
        frontRightDrive.setPower(TURN_SPEED);
        backRightDrive.setPower(TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Go Reload: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Shoot 3 artifacts
        shooterMotor.setPower(1);
        for (int i = 1; i < 3; i++) {
            sleep(30);
            shooterReloader.setPosition(0); // untested
            sleep(10); // untested
            shooterReloader.setPosition(1); // untested
        }
        shooterMotor.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Shooting...: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Move to reload zone
        frontLeftDrive.setPower(TURN_SPEED);
        backLeftDrive.setPower(-TURN_SPEED);
        frontRightDrive.setPower(-TURN_SPEED);
        backRightDrive.setPower(TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Go Reload: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop (wait to be reloaded)
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8.0)) {
            telemetry.addData("Path", "Reloading...: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Move back to launch line
        frontLeftDrive.setPower(-TURN_SPEED);
        backLeftDrive.setPower(TURN_SPEED);
        frontRightDrive.setPower(TURN_SPEED);
        backRightDrive.setPower(-TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Moving to Launch Line: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Stop
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        // current code = 18 second auto
    }
}
