package frc.robot.util;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class GenericPaths {

    public static Waypoint[] path = new Waypoint[]{
            new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
            new Waypoint(6.0, 1.5, Pathfinder.d2r(-45.0)),
    };


}
