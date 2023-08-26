package org.firstinspires.ftc.teamcode.TeleOp.RequiesceToo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="RequiesceToo", group="Cool OP modes")
public class RequiesceToo extends LinearOpMode {

    private final ElapsedTime time = new ElapsedTime();
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    Drive drive = new Drive();
    Arm arm = new Arm();
    Claw claw = new Claw();
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (claw.rClaw.getPosition() >= 0.25 && claw.lClaw.getPosition() < 0.29){
                claw.lClaw.setPosition(0.29);
            }
            drive.Go();
            arm.Go();
            claw.Go();
        }
    }

    private void initialize(){
        drive.Init();
        arm.Init();
        claw.Init();

        telemetry.addLine("Let's up Requiesce"); //It passed the fire test. WOOOH!
        telemetry.update();
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
