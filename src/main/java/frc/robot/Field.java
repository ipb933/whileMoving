package frc.robot;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Field {
    public static final double FIELD_LENGTH = 16.533114; // 54 feet
    public static final double FIELD_WIDTH = 8.069263; // 27 feet

    public Rectangle2d HUMAN_PLAYER_STATION(boolean isRed){
        if(isRed) return Red.HUMAN_PLAYER_STATION;
        return Blue.HUMAN_PLAYER_STATION;
    }

    public Rectangle2d CLIMB(boolean isRed){
        if(isRed) return Red.CLIMB;
        return Blue.CLIMB;
    }

    public Rectangle2d DEPOT_OUTLINE(boolean isRed){
        if(isRed) return Red.DEPOT_OUTLINE;
        return Blue.DEPOT_OUTLINE;
    }

    public Rectangle2d DEPOT_INNER(boolean isRed){
        if(isRed) return Red.DEPOT_INNER;
        return Blue.DEPOT_INNER;
    }

    public Rectangle2d ALLIANCE_LINE(boolean isRed){
        if(isRed) return Red.ALLIANCE_LINE;
        return Blue.ALLIANCE_LINE;
    }

    public Rectangle2d UPPER_TRENCH(boolean isRed){
        if(isRed) return Red.UPPER_TRENCH;
        return Blue.UPPER_TRENCH;
    }

    public Rectangle2d LOWER_TRENCH(boolean isRed){
        if(isRed) return Red.LOWER_TRENCH;
        return Blue.LOWER_TRENCH;
    }

    public Rectangle2d UPPER_BUMP(boolean isRed){
        if(isRed) return Red.UPPER_BUMP;
        return Blue.UPPER_BUMP_LOWER_RIGHT;
    }

    public Rectangle2d LOWER_BUMP(boolean isRed){
        if(isRed) return Red.LOWER_BUMP;
        return Blue.LOWER_BUMP;
    }

    public static Rectangle2d HUB(boolean isRed){
        if(isRed) return Red.HUB;
        return Blue.HUB;
    }

    // Red Alliance Side Field Elements
    public final class Red{
        //Outpost
        public static final Rectangle2d HUMAN_PLAYER_STATION = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 1.265936),
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH)
        );

        //Climb
        public static final Rectangle2d CLIMB = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 3.298031),
            new Translation2d(FIELD_LENGTH - 1.016000, FIELD_WIDTH - 4.193381)
        );
        
        //Depot
        public static final Rectangle2d DEPOT_OUTLINE = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, 2.637631),
            new Translation2d(FIELD_LENGTH - 0.685800, 1.570831)
        );
        public static final Rectangle2d DEPOT_INNER = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, 2.561431),
            new Translation2d(FIELD_LENGTH - 0.609600, 1.820831)
        );
        
        //Alliance Line
        public static final Rectangle2d ALLIANCE_LINE = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, FIELD_LENGTH - 4.003326),
            new Translation2d(0, FIELD_LENGTH - 4.003326)
        );
        
        //Upper Trench
        public static final Rectangle2d UPPER_TRENCH = new Rectangle2d(
            new Translation2d(FIELD_LENGTH, FIELD_WIDTH - 4.625626),
            new Translation2d(FIELD_LENGTH - 1.278731, FIELD_WIDTH - 4.625626)
        );
        
        //Lower Trench
        public static final Rectangle2d LOWER_TRENCH = new Rectangle2d(
            new Translation2d(1.278731, FIELD_WIDTH - 4.625626),
            new Translation2d(0, FIELD_WIDTH - 4.625626)
        );

        //Upper Bump
        public static final Rectangle2d UPPER_BUMP = new Rectangle2d(
            new Translation2d(FIELD_LENGTH - 4.061808, FIELD_WIDTH - 1.583531),
            new Translation2d(FIELD_LENGTH - 5.186157, FIELD_WIDTH - 3.437731)
        );
        
        //Lower Bump
        public static final Rectangle2d LOWER_BUMP = new Rectangle2d(
            new Translation2d(FIELD_LENGTH - 4.061808, 3.437731),
            new Translation2d(FIELD_LENGTH - 5.186157, 1.583531)
        );

        //Hub
        public static final Translation2d HUB_CENTER = new Translation2d(FIELD_LENGTH - 4.6240675, FIELD_WIDTH / 2);
        public static final Rectangle2d HUB = new Rectangle2d(
            new Translation2d(FIELD_LENGTH - 4.039281, (FIELD_WIDTH + 1.1938) / 2),
            new Translation2d(FIELD_LENGTH - 5.221443, (FIELD_WIDTH - 1.1938) / 2)
        );
    }

    // Blue Alliance Side Field Elements
    public final class Blue{
        //Outpost
        public static final Rectangle2d HUMAN_PLAYER_STATION = new Rectangle2d(
            new Translation2d(0, 1.265936),
            new Translation2d(0, 0)
        );
        
        //Climb
        public static final Rectangle2d CLIMB = new Rectangle2d(
            new Translation2d(0, 3.298031),
            new Translation2d(1.016000, 4.193381)
        );

        //Depot
        public static final Rectangle2d DEPOT_OUTLINE = new Rectangle2d(
            new Translation2d(0, FIELD_WIDTH - 2.637631),
            new Translation2d(0.685800, FIELD_WIDTH - 1.570831)
        );
        public static final Rectangle2d DEPOT_INNER = new Rectangle2d(
            new Translation2d(0, FIELD_WIDTH - 2.561431),
            new Translation2d(0.609600, FIELD_WIDTH - 1.820831)
        );
        
        //Alliance Line
        public static final Rectangle2d ALLIANCE_LINE = new Rectangle2d(
            new Translation2d(0, 4.003326),
            new Translation2d(FIELD_LENGTH, 4.003326)
        );
        
        //Upper Trench
        public static final Rectangle2d UPPER_TRENCH = new Rectangle2d(
            new Translation2d(0, 4.625626),
            new Translation2d(1.278731, 4.625626)
        );
        
        //Lower Trench
        public static final Rectangle2d LOWER_TRENCH = new Rectangle2d(
            new Translation2d(FIELD_LENGTH - 1.278731, 4.625626),
            new Translation2d(FIELD_LENGTH, 4.625626)
        );
        
        //Upper Bump
        public static final Rectangle2d UPPER_BUMP_LOWER_RIGHT = new Rectangle2d(
            new Translation2d(4.061808, 1.583531),
            new Translation2d(5.186157, 3.437731)
        );
        
        //Lower Bump
        public static final Rectangle2d LOWER_BUMP = new Rectangle2d(
            new Translation2d(4.061808, FIELD_WIDTH - 3.437731),
            new Translation2d(5.186157, FIELD_WIDTH - 1.583531)
        );

        //Hub
        public static final Translation2d HUB_CENTER = new Translation2d(4.6240675, FIELD_WIDTH / 2);
        public static final Rectangle2d HUB = new Rectangle2d(
            new Translation2d(5.221443, (FIELD_WIDTH + 1.1938) / 2),
            new Translation2d(4.039281, (FIELD_WIDTH - 1.1938) / 2)
        );
    }

    //Balls Field
    public static final Rectangle2d BALLS_FIELD = new Rectangle2d(
        new Translation2d(FIELD_LENGTH - 7.357326, FIELD_WIDTH - 1.724431),
        new Translation2d(7.357326, 1.724431)
    );
}
