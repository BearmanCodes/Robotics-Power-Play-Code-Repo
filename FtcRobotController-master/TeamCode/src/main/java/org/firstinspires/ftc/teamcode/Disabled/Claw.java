package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="Servo", group="Cool OP modes")
public class Claw extends LinearOpMode {

    public Servo servo;
    public Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    public Gamepad previousGamepad = new Gamepad();


    @Override
    public void runOpMode() {
        initialize(); //Configure and setup all the hardware, among other things

        waitForStart(); //Start the actual program once we confirm the robot isn't on fire

        //Do everything in these blocks until we press stop. Or something crashes.
        while (opModeIsActive()) {
            servo.setPosition(0);
            if (gamepad1.a){
                servo.setPosition(0.1);
            }
            if (gamepad1.b){
                servo.setPosition(0);
            }
        }
    }

    private void initialize(){
        servo = hardwareMap.get(Servo.class, "claw");

    }

    private void edgeDetector(){
        try{
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
        } catch(RobotCoreException e){
            telemetry.addLine("If you're seeing this, the program is trying to use a ga+"+
                    "mepad that does not exist. Fix it would ya?");
            telemetry.update();
        }
    }
}
