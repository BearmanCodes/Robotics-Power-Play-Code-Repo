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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Stack", group="Auto")
public class Stack extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx arm;
    int ticks = 0;
    Servo lClaw;
    Servo rClaw;
    BNO055IMU imu;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();


    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);

    @Override
    public void runOpMode() {


        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armMove(4500, 1000, 50); //Reach
        clawGrab(); //Grab
        sleep(500);
        armMove(4500, 2700, 50); //Lift up
        goBack();

        armMove(4500, 875, 50); //Reach
        clawGrab(); //Grab
        sleep(500);
        armMove(4500, 2400, 50); //Lift up
        goBack();

        armMove(4500, 400, 50); //Reach
        clawGrab(); //Grab
        sleep(500);
        armMove(4500, 1800, 50); //Lift up
        goBack();

        armMove(4500, 50, 50); //Reach
        clawGrab(); //Grab
        sleep(500);
        armMove(4500, 1450, 50); //Lift up
        goBack();

        armMove(4500, 0, 50); //Reach
        clawGrab(); //Grab
        sleep(500);
        armMove(4500, 1450, 50); //Lift up
        goBack();


        // 5th cone being the very top of the stack and 1st cone being the very bottom of the stack
        //1000 ticks to reach 5th cone. 2700/3000 ticks to lift the 5th cone off
        //900 (bit lower maybe) ticks to reach 4th cone. 2400 ticks to lift the 4th cone off
        //400 ticks to reach the 3rd cone. 1800 ticks to lift the 3rd cone off
        //50 ticks (can be a bit higher) to reach the 2nd cone. 1450 ticks to lift the 2nd cone off
        //0 ticks (ground floor) to reach the 1st cone. pretty much any amount of ticks to lift the 1st cone off
        //super helpful drive diagram https://gm0.org/en/latest/_images/mecanum-drive-directions.png


    }

    public void goBack(){
        Drive(850, -20, -20, -20, -20, 50); //Reverse
        Drive(450, -10, 10, -10, 10, 50); //Turn left
        clawOpen(); //Release
        sleep(500);
        Drive(450, 10, -10, 10, -10, 50); //Turn right to get back on track
        Drive(850, 20, 20, 20, 20, 50); //Forward
    }


    public void Drive(double velocity,
                      double frontLeftInches, double frontRightInches,
                      double backLeftInches, double backRightInches,
                      long timeout) {
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * TicksPerIn);
            frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * TicksPerIn);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * TicksPerIn);
            backRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * TicksPerIn);

            allTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            allMotorVelocity(Math.abs(velocity));

            while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addLine("WE ARE MOVING, WOOOOO!");
                telemetry.update();
            }

            allMotorVelocity(0);

            allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(timeout);
        }
    }


    private void initialize() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //Reverses it's direction so that it goes the right way.
        rClaw.setDirection(Servo.Direction.REVERSE);
        lClaw.setPosition(0.1);
        rClaw.setPosition(0);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.


        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void allMotorMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void allMotorVelocity(double velocity) {
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);
    }

    public void allTargetPosition(int frontLeftPos, int frontRightPos,
                                  int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }

    public void armMove(double velocity, int ticks, int timeout){
        arm.setTargetPosition(ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        while (opModeIsActive() && arm.isBusy()){
            telemetry.addData("Position: ", arm.getCurrentPosition());
            telemetry.addData("Target: ", ticks);
            telemetry.update();
        }
        arm.setVelocity(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(timeout);
    }

    public void clawOpen(){
        lClaw.setPosition(0.1);
        rClaw.setPosition(0);
    }

    public void clawGrab(){
        lClaw.setPosition(0.35);
        rClaw.setPosition(0.28);
    }

    private void ClawGo(){
        //Left claw, 0.1 opens it up, 0.3 closes it enough, 0.35/0.4 to really clamp.
        //Right claw, 0.0 opens it up, 0.2 closes it just a bit, 0.25/0.3 to really close.
        if (currentGamepad.right_bumper && !previousGamepad.right_bumper){ //Change to A once I get back
            lClaw.setPosition(0.35);
            rClaw.setPosition(0.28);
        }
        if (currentGamepad.left_bumper && !previousGamepad.left_bumper){
            lClaw.setPosition(0.1);
            rClaw.setPosition(0);
        }
    }

}

