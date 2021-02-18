package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;



import com.qualcomm.robotcore.hardware.DcMotor;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;

public class LinearSlidesMotorsSubsystem extends SubSystem {
    CustomizableGamepad input;
    DcMotor rightM;
    DcMotor leftM;
    public LinearSlidesMotorsSubsystem(Robot r, String rightMotor, String leftMotor) {
        super(r);
        rightM = robot.hardwareMap.dcMotor.get(rightMotor);
        leftM = robot.hardwareMap.dcMotor.get(leftMotor);
        usesConfig = true;
    }
    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        input = robot.pullControls(this);
    }

    @Override
    public void handle() {
        if(input.getInput("DownButton")) {
            rightM.setPower(1);
            leftM.setPower(-1);
        }
        else if(input.getInput("UpButton")) {
            rightM.setPower(-1);
            leftM.setPower(1);
        }
        else {
            rightM.setPower(0);
            leftM.setPower(0);
        }
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("UpButton", Button.BooleanInputs.bool_right_trigger,2),
                new ConfigParam("DownButton", Button.BooleanInputs.bool_left_trigger,2)
        };
    }
}
