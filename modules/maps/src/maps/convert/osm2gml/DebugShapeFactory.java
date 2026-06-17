package maps.convert.osm2gml;

import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.gui.ShapeDebugFrame;

import java.awt.*;

public final class DebugShapeFactory {

    private static final Color COLOR_OUTLINE      = Color.BLACK;
    private static final Color COLOR_CENTROID     = Color.BLACK;
    private static final Color COLOR_PASSABLE     = Color.GREEN;
    private static final Color COLOR_BLOCKED_LINE = Color.RED;

    private static final Color FILL_OUTLINE       = Color.LIGHT_GRAY;

    public static ShapeDebugFrame.ShapeInfo createOutlineInfo(final TemporaryObject object) {
        return new TemporaryObjectInfo(object, "Object Outline", COLOR_OUTLINE, FILL_OUTLINE);
    }

    public static ShapeDebugFrame.ShapeInfo createCentroidInfo(final TemporaryObject object) {
        return new ShapeDebugFrame.Point2DShapeInfo(
                object.getCentroid(), "Object Centroid", COLOR_CENTROID, false);
    }

    public static ShapeDebugFrame.ShapeInfo createTraversalLineInfo(
            final Line2D traversalLine, final boolean isPassable) {
        final Color  color = isPassable ? COLOR_PASSABLE  : COLOR_BLOCKED_LINE;
        final String label = isPassable ? "Passable Line" : "Blocked Line";
        return new ShapeDebugFrame.Line2DShapeInfo(traversalLine, label, color, false, false);
    }

}
