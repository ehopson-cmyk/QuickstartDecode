package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BasicPPAuto extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private DcMotorEx intake; 

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {
    public PathChain Path1;
    public PathChain Path2;

    public Paths(Follower follower) {
      Path1 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(56.000, 8.000),
            new Pose(56.000, 87.238)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
          .build();

      Path2 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(56.000, 87.238),
            new Pose(105.307, 33.479)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
          .build();
    }
  }

  public void autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    switch (pathState) {
        case 0:
            follower.followPath(paths.Path1);
            setPathState(1);
            break;
        case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if(!follower.isBusy()) {
                intake.setPower(1);
                sleep(1000);
                intake.setPower(0);
                follower.followPath(paths.Path2,true);
                setPathState(2);
            }
            break;
    return 0;
  }
}
