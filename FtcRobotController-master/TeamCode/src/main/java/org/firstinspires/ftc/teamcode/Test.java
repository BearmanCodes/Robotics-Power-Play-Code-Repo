package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;
@TeleOp(name = "Servo Testy", group = "tests")
public class Test extends LinearOpMode {
    public CRServo crServo;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){
        crServo = hardwareMap.get(CRServo.class, "servo");
        crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                crServo.setPower(1);
            }
            if (gamepad1.b){
                crServo.setPower(-1);
            }
            if (gamepad1.x){
                crServo.setPower(0);
            }
        }
    }
}
