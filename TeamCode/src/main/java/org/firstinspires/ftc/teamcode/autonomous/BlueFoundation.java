package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;

/**
 * Created by Silver Titans on 09/28/2019
 * <p>
 * Here's our approach for the autonomous period
 * Steps
 * 1. Find Sky-stone in top section of quarry
 * 2. Approach top sky-stone
 * 3. Capture sky-stone
 * 3. Navigate to foundation
 * 5. Place stone
 * 6. Approach bottom sky-stone
 * 7. Capture second sky-stone
 * 8. Navigate to foundation
 * 9. Place stone
 * 10. Move foundation
 * 11. Park
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Foundation", group="Phoebe")
//@Disabled
public class BlueFoundation extends AutonomousHelperFoundation {

    @Override
    public void init() {
        super.init(Alliance.Color.BLUE);
    }
}