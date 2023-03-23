package org.firstinspires.ftc.teamcode.THISIS10111.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.Robot10111;

public class ArmUp extends CommandBase {
    //a command to lift arm to certain height using ArmObj functions

    //local vars for target height and armobj
    private final ArmObj armobj;
    private final ArmHeights height;

    public ArmUp(ArmObj armObj, ArmHeights height) {
        this.armobj = armObj; //sets the local armobj var to the ArmObj of the robot
        this.height = height; //sets the local height var to the required height

        addRequirements(armObj); //requires an armobj to run
    }

    @Override
    public void initialize() {
        //sets the armobj target height to "height"
        armobj.setHeight(height);
    }
    @Override
    public boolean isFinished() {
        //if the difference between the target and current heights (error) is less than 50, the command is complete
        Robot10111 robot = Robot10111.getInstance();

        return Math.abs(armobj.armGet().getTargetPosition() - robot.arm.getEncoderCount()) < 50;
    }
}
