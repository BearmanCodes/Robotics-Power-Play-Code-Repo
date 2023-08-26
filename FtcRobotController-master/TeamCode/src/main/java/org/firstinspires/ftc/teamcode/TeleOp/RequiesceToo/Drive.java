package org.firstinspires.ftc.teamcode.TeleOp.RequiesceToo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
public class Drive extends LinearOpMode {
    double Vertical, Horizontal, Pivot, fLpower, fRpower, bRpower, bLpower;
    double reducer = 0.75;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    public void Go(){
        Vertical = gamepad1.left_stick_y;
        Horizontal = gamepad1.left_stick_x * 1.2;
        Pivot = gamepad1.right_stick_x;
        fLpower = (-Pivot + (Vertical - Horizontal)) * reducer;
        fRpower = (Pivot + Vertical + Horizontal) * reducer;
        bRpower = (Pivot + (Vertical - Horizontal)) * reducer;
        bLpower = (-Pivot + Vertical + Horizontal) * reducer;

        frontLeft.setPower(fLpower);
        frontRight.setPower(fRpower);
        backLeft.setPower(bLpower);
        backRight.setPower(bRpower);
    }

    public void Init(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //This is just needed because annoying stuff
    public void runOpMode() throws InterruptedException {
    }
}
