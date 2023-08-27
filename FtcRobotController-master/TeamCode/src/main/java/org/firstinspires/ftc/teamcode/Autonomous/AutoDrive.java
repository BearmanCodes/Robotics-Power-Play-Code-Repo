package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;
@Disabled
public class AutoDrive extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);

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

    public void Init(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void allTargetPosition(int frontLeftPos, int frontRightPos,
                                  int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
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

    public void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower,
                              double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void allMotorPower(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void strafeNW(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, 0, inches, inches, 0, tOut);
    }

    public void fwdDrive(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, inches, inches, inches, inches, tOut);
    }

    public void strafeNE(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, inches, 0, 0, inches, tOut);
    }

    public void strafeLeft(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, -inches, inches, inches, -inches, tOut);
    }

    public void strafeRight(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, inches, -inches, -inches, inches, tOut);
    }

    public void turnCC(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, -inches, inches, -inches, inches, tOut);
    }

    public void turnCW(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, inches, -inches, inches, -inches, tOut);
    }

    public void strafeSW(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, -inches, 0, 0, -inches, tOut);
    }

    public void revDrive(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, -inches, -inches, -inches, -inches, tOut);
    }

    public void strafeSE(double vel, double inches, long tOut){
        inches = Math.abs(inches);
        Drive(vel, 0, -inches, -inches, 0, tOut);
    }


    public void runOpMode() throws InterruptedException {
    }
}
