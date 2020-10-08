package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.TurnOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

/**
Go backwards 15 inches
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Test turn", group="Phoebe")
//@Disabled
public class Test extends AutonomousHelperNavigate {
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
        robot.queuePrimaryOperation(new TurnOperation(90, 0.5, TurnOperation.Direction.LEFT, "Turn Left 90"));
        robot.queuePrimaryOperation(new WaitOperation(2000, "Wait"));
        robot.queuePrimaryOperation(new TurnOperation(-90, 0.5, TurnOperation.Direction.LEFT, "Turn Left -90"));
        robot.queuePrimaryOperation(new WaitOperation(2000, "Wait"));

        robot.queuePrimaryOperation(new TurnOperation(90, 0.5, TurnOperation.Direction.RIGHT, "Turn right 90"));
        robot.queuePrimaryOperation(new WaitOperation(2000, "Wait"));
        robot.queuePrimaryOperation(new TurnOperation(-90, 0.5, TurnOperation.Direction.RIGHT, "Turn right -90"));
    }

}