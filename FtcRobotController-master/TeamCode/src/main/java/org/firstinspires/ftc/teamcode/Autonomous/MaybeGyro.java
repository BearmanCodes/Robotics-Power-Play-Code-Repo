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

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.ObjectTargetResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.security.AlgorithmConstraints;

@Autonomous(name="Gyro", group="Auto")
public class MaybeGyro extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx arm;
    ColorSensor colorSensor;
    int red;
    int green;
    int blue;
    BNO055IMU imu;

    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);
    Orientation lastAngle = new Orientation();
    double currentAngle = 0;

    @Override
    public void runOpMode() {


        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //super helpful drive diagram https://gm0.org/en/latest/_images/mecanum-drive-directions.png
        arm.setPower(1);
        long start = System.currentTimeMillis();
        long end = start + 300;
        while (System.currentTimeMillis() < end){
            telemetry.addLine("Tightening");
            telemetry.update();
        }
        arm.setPower(0);
        sleep(500);
        Drive(3500, 33, 33, 33, 33, 100);
        turnTo(90);
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
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        arm = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //Reverses it's direction so that it goes the right way.
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSensor.enableLed(false);
        colorLoad();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void allMotorPower(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower,
                              double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void allTargetPosition(int frontLeftPos, int frontRightPos,
                                  int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
    }

    public void colorLoad(){
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
    }

    public void telemetryColor(){
        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);
        telemetry.update();
    }

    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= - 180){
            deltaAngle += 360;
        }
        currentAngle += deltaAngle;
        lastAngle = orientation;
        return currentAngle;
    }

    public void turnCC(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 1){
            double power = (error < 0 ? -0.2 : 0.2);
            setMotorPower(-power, power, -power, power);
            error = degrees - getAngle();
            telemetry.addData("error: ", error);
            telemetry.update();
        }

        allMotorPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error <= - 180){
            error += 360;
        }
        turnCC(error);
    }
}

