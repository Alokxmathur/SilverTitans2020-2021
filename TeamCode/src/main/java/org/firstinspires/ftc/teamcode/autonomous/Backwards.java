package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;

/**
Go backwards 15 inches
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Backwards 15\"", group="Phoebe")
//@Disabled
public class Backwards extends AutonomousHelperNavigate {
    @Override
    public void init() {
        super.init(Alliance.Color.RED);
    }

    /**
     * Move backwards by 15 inches
     */
    @Override
    protected void queueInitialOperations() {
        //queue backward movement towards tape
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-15* Field.MM_PER_INCH, 0, SPEED, "Move towards tape"));
    }

}