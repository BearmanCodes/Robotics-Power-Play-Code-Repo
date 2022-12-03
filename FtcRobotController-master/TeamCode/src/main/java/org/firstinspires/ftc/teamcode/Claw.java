package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo", group="Cool OP modes")
public class Claw extends LinearOpMode {

    Servo servo;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    DcMotorEx coreHex;
    double hexPower;


    @Override
    public void runOpMode() {
        initialize(); //Configure and setup all the hardware, among other things

        waitForStart(); //Start the actual program once we confirm the robot isn't on fire

        //Do everything in these blocks until we press stop. Or something crashes.
        while (opModeIsActive()) {
            edgeDetector();
            hexPower = -gamepad1.left_trigger + gamepad1.right_trigger;
            coreHex.setPower(hexPower);
            if (currentGamepad.a && !previousGamepad.a){
                servo.setPosition(servo.getPosition() + 0.05);
            }
            if (currentGamepad.b && !previousGamepad.b){
                servo.setPosition(servo.getPosition() - 0.05);
            }
            telemetry.addData("Position: ", servo.getPosition());
            telemetry.update();
        }
    }

    private void initialize(){
        servo = hardwareMap.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.FORWARD);
        coreHex = hardwareMap.get(DcMotorEx.class, "hex");
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
