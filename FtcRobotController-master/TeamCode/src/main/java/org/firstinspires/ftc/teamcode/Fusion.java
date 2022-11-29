package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Awesome Drive", group="Cool OP modes")
public class Fusion extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx arm;
    private DcMotorEx extend;
    DistanceSensor colorSensor;
    double distance;
    double extendPower;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    BNO055IMU imu;
    double backRightPower;
    double frontRightPower;
    double backLeftPower;
    double frontLeftPower;
    double Vertical;
    double Horizontal;
    double Pivot;



    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            Vertical = gamepad1.left_stick_y;
            Horizontal = gamepad1.left_stick_x * 1.4;
            Pivot = gamepad1.right_stick_x;

            frontLeftPower = (-Pivot + (Vertical - Horizontal)) * 0.65;
            frontRightPower = (Pivot + Vertical + Horizontal) * 0.65;
            backRightPower = (Pivot + (Vertical - Horizontal)) * 0.65;
            backLeftPower = (-Pivot + Vertical + Horizontal) * 0.605;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            */
            distance = colorSensor.getDistance(DistanceUnit.CM); //Update the color sensor distance right off the bat.
            extend.setPower(extendPower); //Set the extendable arm motor power to the variable.
            extendPower = -gamepad1.left_trigger * 0.45 + gamepad1.right_trigger * 0.45; //Set the power variable to the triggers.
            if (gamepad1.a) { //Do everything in here once you press A.
                arm.setPower(1); //Go full ham on the motor. (Wish I could use velocity but the encoder on the 40:1 is brokey)
                telemetry.addLine("It's a-moving!");
                telemetry.update();
                long start = System.currentTimeMillis(); //Declare and then set a variable to the current time
                long end = start + 1900; //Declare and then set a variable to what time to end (1900 MS after)
                while (System.currentTimeMillis() < end){ //While it still hasn't been 1900 MS since.
                    if (gamepad1.x) { //Press X to stop the motor in case something bad happens in between 1900 MS. The entire reason I used System instead of sleep.
                        arm.setPower(0);
                        break;
                    }
                    telemetry.addLine("It's a-moving!");
                    telemetry.addLine("Press X if things go Wacko!!!!!!!!!!!");
                    telemetry.update();
                }
                arm.setPower(0); //Stop the motor after it's moved for 1900 MS.
                telemetry.addLine("DONE!");
                telemetry.update();
            }
            if (gamepad1.b) { //Do everything in here once you press B.
                if (distance > 4.2) { //Makes sure it can only reverse if the cascading kit has moved up.
                    arm.setPower(-1); //Reverse, reverse. Cha cha now y'all.
                    distance = colorSensor.getDistance(DistanceUnit.CM); //Update the distance.
                    while (opModeIsActive() && distance > 4.2) { //While the cascading kit is still up do everything in here.
                        if (gamepad1.x) { //Press X to stop the motor in case everything goes WACKO!
                            arm.setPower(0);
                            break;
                        }
                        distance = colorSensor.getDistance(DistanceUnit.CM); //Update the distance within the loop, otherwise it'll just keep cha cha-ing.
                        telemetry.addLine("It's Cha Cha-ing!");
                        telemetry.update();
                    }
                    arm.setPower(0); //Stop once the cascading kit is down.
                }
            }


            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.2;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double frontLeftPower = (rotY + rotX + rx) * 0.75;
            double backLeftPower = (rotY - rotX + rx) * 0.75;
            double frontRightPower = (rotY - rotX - rx) * 0.75;
            double backRightPower = (rotY + rotX - rx) * 0.75;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }

    private void initialize(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotorEx.class, "arm"); //Assign the cascading kit motor.
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.
        arm.setDirection(DcMotorSimple.Direction.REVERSE); //Reverses it's direction so that it goes the right way.
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); //Assign the color sensor.
        extend = hardwareMap.get(DcMotorEx.class, "extend"); //Assign the extendable arm motor.
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Set it to Brake like the cascading kit.
        extend.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse it's direction, subject to change.
        //These lines tighten the string just to make sure it's in the spool like it should be.
        arm.setPower(0.2);
        sleep(100);
        arm.setPower(0);
        telemetry.addLine("It didn't catch on fire YAY!"); //It passed the fire test. WOOOH!
        telemetry.update();
    }
}
