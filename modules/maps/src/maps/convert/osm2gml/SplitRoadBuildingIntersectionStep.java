package maps.convert.osm2gml;

import rescuecore2.misc.geometry.Point2D;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * This step splits road and building edges at their mutual intersection points,
 * ensuring that both structures share exact nodes at crossing locations.
 */
public class SplitRoadBuildingIntersectionStep extends BaseModificationStep {

    public SplitRoadBuildingIntersectionStep(final TemporaryMap map) {
        super(map);
    }

    @Override
    public String getDescription() {
        return "Splitting road and building edges at intersections";
    }

    @Override
    protected void step() {
        final List<TemporaryObject> passableShapes = new ArrayList<>(map.getAllPassableShapes());
        final List<TemporaryBuilding> buildings      = new ArrayList<>(map.getBuildings());

        if (passableShapes.isEmpty() || buildings.isEmpty()) {
            setStatus("No roads or buildings to process.");
            return;
        }

        int splitCount = 0;
        setProgressLimit(passableShapes.size());

        for (final TemporaryObject shape : passableShapes) {
            for (final TemporaryBuilding building : buildings) {
                // Quick bounds check for performance.
                if (!shape.getBounds().intersects(building.getBounds())) {
                    continue;
                }

                splitCount += splitAtIntersections(shape, building);
            }
            bumpProgress();
        }

        setStatus("Split " + splitCount + " edges at road/building intersections.");
    }

    // Split road and building edges at all mutual intersection points.
    // Returns the total number of split operations performed.
    private int splitAtIntersections(final TemporaryObject road, final TemporaryBuilding building) {
        final List<Edge> roadEdges     = new ArrayList<>(ConvertTools.collectEdges(road));
        final List<Edge> buildingEdges = new ArrayList<>(ConvertTools.collectEdges(building));

        // Collect all intersection nodes before any splits occur,
        // so that edge snapshots remain valid throughout collection.
        final Map<Edge, List<Node>> roadSplitNodes     = new LinkedHashMap<>();
        final Map<Edge, List<Node>> buildingSplitNodes = new LinkedHashMap<>();

        for (final Edge roadEdge : roadEdges) {
            for (final Edge buildingEdge : buildingEdges) {
                final Node splitNode = findIntersectionNode(roadEdge, buildingEdge);
                if (splitNode == null) {
                    continue;
                }

                // Register the split node for each edge that does not already have is an endpoint.
                if (ConvertTools.isInteriorNode(splitNode, roadEdge)) {
                    roadSplitNodes.computeIfAbsent(roadEdge, e -> new ArrayList<>()).add(splitNode);
                }
                if (ConvertTools.isInteriorNode(splitNode, buildingEdge)) {
                    buildingSplitNodes.computeIfAbsent(buildingEdge, e -> new ArrayList<>()).add(splitNode);
                }
            }
        }

        // Apply all splits after collection is complete.
        roadSplitNodes.forEach(map::splitEdge);
        buildingSplitNodes.forEach(map::splitEdge);

        return roadSplitNodes.size() + buildingSplitNodes.size();
    }

    // Find the intersection node between two edges, or null if none exists.
    // Handles near-miss cases where the intersection lies just outside a segment
    // due to floating-point error, using an infinite-line fallback.
    private Node findIntersectionNode(final Edge roadEdge, final Edge buildingEdge) {
        final Point2D intersection =
                ConvertTools.findInteriorIntersection(roadEdge, buildingEdge, map);
        if (intersection == null) {
            return null;
        }
        return map.getNode(intersection);
    }
}
