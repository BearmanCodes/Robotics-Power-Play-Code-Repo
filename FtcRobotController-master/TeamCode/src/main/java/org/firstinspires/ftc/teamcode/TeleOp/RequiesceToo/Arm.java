package org.firstinspires.ftc.teamcode.TeleOp.RequiesceToo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
public class Arm extends LinearOpMode {
    double cascadePower;
    DcMotorEx arm;
    public void Go(){
        cascadePower = -gamepad2.left_trigger + gamepad2.right_trigger;
        arm.setPower(cascadePower);
    }

    public void Init(){
        arm = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //Reverses it's direction so that it goes the right way.
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //These lines tighten the string just to make sure it's in the spool like it should be.
        arm.setPower(0.2);
        long start = System.currentTimeMillis();
        long end = start + 250;
        while (System.currentTimeMillis() < end){
            telemetry.addLine("Tightening");
            telemetry.update();
        }
        arm.setPower(0);
    }

    //Needed because annoying stuff
    public void runOpMode() throws InterruptedException {
    }
}
