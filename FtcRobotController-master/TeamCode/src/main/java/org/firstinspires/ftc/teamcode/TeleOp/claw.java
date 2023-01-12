package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ServoTester", group="tests")
public class claw extends LinearOpMode {
    Servo lClaw;
    Servo rClaw;
    Servo flipper;
    Gamepad previousGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();
    double lpos;
    double rpos;
    double flipPos;

    //Left claw, 0.1 opens it up, 0.3 closes it enough, 0.35/0.4 to really clamp.
    //Right claw, 0.0 opens it up, 0.2 closes it just a bit, 0.25/0.3 to really close.

    @Override
    public void runOpMode(){
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            try {
                edgeDetector();
            } catch (RobotCoreException e){
                telemetry.addLine("SOMETHING WENT WRONG");
                telemetry.update();
            }
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper){
                flipPos += 0.01;
            }
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper){
                flipPos -= 0.01;
            }
            if (currentGamepad.a && !previousGamepad.a){ //Change to A once I get back
                lpos += 0.01;
                output();
            }
            if (currentGamepad.b && !previousGamepad.b){
                lpos -= 0.01;
                output();
            }
            if (currentGamepad.x && !previousGamepad.x){
                rpos += 0.01;
                output();
            }
            if (currentGamepad.y && !previousGamepad.y){
                rpos -= 0.01;
                output();
            }
            if (currentGamepad.start && !previousGamepad.start){
                lClaw.setPosition(lpos);
                rClaw.setPosition(rpos);
                flipper.setPosition(flipPos);
                output();
            }
            output();
        }

    }

    public void initialize(){
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");
        flipper = hardwareMap.get(Servo.class, "flip");
        rClaw.setDirection(Servo.Direction.REVERSE);
        lClaw.setPosition(0.2);
        rClaw.setPosition(0);
    }

    public void edgeDetector() throws RobotCoreException{
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }

    public void output(){
        telemetry.addData("Left Pos: ", lpos);
        telemetry.addData("Right Pos: ", rpos);
        telemetry.addData("Flipper Pos: ", flipPos);
        telemetry.addLine("-------------------------------------------------");
        telemetry.addData("Real Left Pos: ", lClaw.getPosition());
        telemetry.addData("Real Right Pos: ", rClaw.getPosition());
        telemetry.addData("Real Flipper Pos: ", flipper.getPosition());
        telemetry.update();
    }
}
