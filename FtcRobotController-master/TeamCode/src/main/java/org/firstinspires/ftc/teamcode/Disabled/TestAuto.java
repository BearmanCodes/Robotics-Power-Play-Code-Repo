package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "AHHH AUTO", group = "tests")
public class TestAuto extends LinearOpMode {
    static final double TOLERANCE = 10;
    int pos = 0;
    public DcMotorEx tetrix;
    int target = 8100;
    double TPS = 4500;
    int error = 240;

    @Override
    public void runOpMode(){
        tetrix = hardwareMap.get(DcMotorEx.class, "motor");
        //tetrix.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tetrix.setDirection(DcMotorSimple.Direction.FORWARD);
        tetrix.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive() && Math.abs(error) > TOLERANCE) {
            pos = tetrix.getCurrentPosition();
            error = target - pos;
            if (error > 0) {
                tetrix.setVelocity(TPS);
            } else if (error < 0) {
                tetrix.setVelocity(-TPS);
            }
            output();
        }
            tetrix.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            tetrix.setVelocity(0);
            pos = tetrix.getCurrentPosition();
            error = target - pos;
            output();
    }
    public void output(){
        telemetry.addData("position: ", pos);
        //telemetry.addData("position: ", tetrix.getCurrentPosition());
        telemetry.addData("target: ", target);
        telemetry.addData("error: ", error);
        telemetry.update();
    }
}
