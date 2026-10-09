package maps.convert.osm2gml;

import maps.convert.ConvertStep;
import maps.convert.osm2gml.debug.DebugPalette;
import maps.convert.osm2gml.debug.PolygonLayer;
import maps.convert.osm2gml.debug.StepVisualizer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;

import java.awt.geom.Area;
import java.util.*;

/**
 * Creates entrance roads serving as building entrances to connect
 * building with adjacent roads.
 */
public class CreateEntrancesStep extends ConvertStep {
    private final TemporaryMap map;

    private final EntranceStrategy strategy;
    private final double maxConnectDistance;
    private final double minConnectDistance;
    private final double maxAngleDeviation;
    private final double entranceWidth;

    /**
     * Strategy for choosing where an entrance attaches to a road.
     */
    public enum EntranceStrategy {
        /**
         * Connects the wall midpoint to the closest point on each nearby road edge, discards
         * candidates exceeding the angle tolerance, and selects the candidate whose center line
         * is closest to perpendicular to both the wall and the road edge.
         */
        MIN_ANGLE_DEVIATION,

        /**
         * Drops a perpendicular from the wall midpoint and selects the road edge it reaches
         * with the shortest entrance. No angle constraint is applied.
         *
         * <p>This is the method adopted by Hosoya et al. (2019), "Map Creations with OpenStreetMap
         * for RoboCupRescue Simulation", 2019 6th International Conference on Computational
         * Science/Intelligence and Applied Informatics (CSII), pp. 60-65.
         * DOI: <a href="https://doi.org/10.1109/CSII.2019.00018">10.1109/CSII.2019.00018</a>
         */
        NEAREST_PERPENDICULAR
    }

    private record EntrancePlan(
            TemporaryIntersection entranceObject,
            Edge buildingEdge, Edge roadEdge,
            Node buildingNode1, Node buildingNode2,
            Node roadNode1, Node roadNode2
    ) {}

    private static final Logger LOGGER = LoggerFactory.getLogger(CreateEntrancesStep.class);

    // Outcome of the entrance creation attempt for a single building
    private enum EntranceResult { ALREADY_CONNECTED, CONNECTED, NOT_CONNECTED }

    // Reasons a candidate pair of building edge and road edge is rejected, declared in evaluation order
    private enum RejectReason {
        ROAD_EDGE_OCCUPIED,
        ROAD_EDGE_TOO_SHORT,
        NO_CONNECTING_POINT,
        ROAD_BEHIND_WALL,
        WALL_BEHIND_ROAD_EDGE,
        DEGENERATE_ENTRANCE,
        CROSSES_OWN_GEOMETRY,
        TOO_SHORT,
        TOO_LONG,
        ANGLE_DEVIATION,
        COLLISION
    }

    private record CandidatePair(DirectedEdge buildingEdge, DirectedEdge roadEdge, TemporaryRoad road) {}

    private record Evaluation(
            EntrancePlan plan, RejectReason reason, CandidatePair pair,
            double entranceLength, double angleDeviation, TemporaryObject collidedWith) {

        static Evaluation accepted(CandidatePair pair, EntrancePlan plan, double entranceLength, double angleDeviation) {
            return new Evaluation(plan, null, pair, entranceLength, angleDeviation, null);
        }

        static Evaluation rejected(CandidatePair pair, RejectReason reason) {
            return rejected(pair, reason, Double.NaN, Double.NaN);
        }

        static Evaluation rejected(CandidatePair pair, RejectReason reason, double entranceLength, double angleDeviation) {
            return new Evaluation(null, reason, pair, entranceLength, angleDeviation, null);
        }

        static Evaluation collided(CandidatePair pair, double entranceLength, double angleDeviation, TemporaryObject other) {
            return new Evaluation(null, RejectReason.COLLISION, pair, entranceLength, angleDeviation, other);
        }

        boolean isAccepted() {
            return plan != null;
        }
    }

    /**
     * Constructs a new {@code CreateEntrancesStep} using {@link EntranceStrategy#MIN_ANGLE_DEVIATION}.
     *
     * @param map the map
     */
    public CreateEntrancesStep(TemporaryMap map) {
        this(map, EntranceStrategy.MIN_ANGLE_DEVIATION);
    }

    /**
     * Constructs a new {@code CreateEntrancesStep}.
     *
     * @param map the map
     * @param strategy the strategy used to choose where each entrance attaches to a road
     */
    public CreateEntrancesStep(TemporaryMap map, EntranceStrategy strategy) {
        this.map = map;
        this.strategy = strategy;
        maxConnectDistance = ConvertTools.sizeOfMeters(map.getOSMMap(), 20);
        minConnectDistance = ConvertTools.sizeOfMeters(map.getOSMMap(), 1); // Nearby threshold
        maxAngleDeviation = 45;
        entranceWidth = ConvertTools.sizeOfMeters(map.getOSMMap(), Constants.ROAD_WIDTH);
    }

    @Override
    public String getDescription() {
        return "Creating roads serving as building entrance";
    }

    @Override
    protected void step() {
        List<TemporaryBuilding> buildings = new ArrayList<>(map.getBuildings());
        List<TemporaryIntersection> entrance = new ArrayList<>();
        Map<EntranceResult, Integer> resultCounts = new EnumMap<>(EntranceResult.class);
        setProgressLimit(buildings.size());

        double cellSize = maxConnectDistance * 1.2;
        SpatialGrid<TemporaryObject> objectGrid = new SpatialGrid<>(map.getBounds(), cellSize);
        map.getAllObjects().forEach(objectGrid::add);

        for (TemporaryBuilding building : buildings) {
            if (isAlreadyConnected(building, map.getRoads())) {
                recordResult(building, EntranceResult.ALREADY_CONNECTED, resultCounts);
                bumpProgress();
                continue;
            }

            EntrancePlan bestPlan = findBestPlanForBuilding(building, objectGrid);
            if (bestPlan == null) {
                recordResult(building, EntranceResult.NOT_CONNECTED, resultCounts);
                bumpProgress();
                continue;
            }

            map.splitEdge(bestPlan.buildingEdge(), bestPlan.buildingNode1(), bestPlan.buildingNode2());
            map.splitEdge(bestPlan.roadEdge(), bestPlan.roadNode1(), bestPlan.roadNode2());
            map.addIntersection(bestPlan.entranceObject());
            entrance.add(bestPlan.entranceObject());
            recordResult(building, EntranceResult.CONNECTED, resultCounts);
            bumpProgress();
        }

        if (!entrance.isEmpty()) {
            map.resynchronizeStateFromObjects();
        }

        setProgress(buildings.size());
        setStatus("Created " + entrance.size() + " new entrances for buildings.");
        logSummary(buildings.size(), resultCounts);
        visualizeResults(entrance);
    }

    private boolean isAlreadyConnected(TemporaryBuilding building, Collection<TemporaryRoad> roads) {
        Set<Edge> buildingEdges = new HashSet<>();
        for (DirectedEdge de : building.getEdges()) {
            buildingEdges.add(de.getEdge());
        }
        for (TemporaryRoad road : roads) {
            for (DirectedEdge de : road.getEdges()) {
                if (buildingEdges.contains(de.getEdge())) {
                    return true;
                }
            }
        }

        return false;
    }

    private EntrancePlan findBestPlanForBuilding(
            TemporaryBuilding building, SpatialGrid<TemporaryObject> objectGrid) {
        EntrancePlan bestPlan = null;
        double bestScore = Double.MAX_VALUE;
        boolean isBuildingCCW = GeometryTools2D.isCounterClockwise(building.getVertices());

        for (DirectedEdge buildingEdge : building.getEdges()) {
            if (buildingEdge.getLength() < entranceWidth) continue;

            for (TemporaryObject object : objectGrid.getNearbyItems(building)) {
                if (!(object instanceof TemporaryRoad road)) continue;

                boolean isRoadCCW = GeometryTools2D.isCounterClockwise(road.getVertices());

                for (DirectedEdge roadEdge : road.getEdges()) {
                    CandidatePair pair = new CandidatePair(buildingEdge, roadEdge, road);
                    Evaluation evaluation = evaluateCandidate(building, pair, isBuildingCCW, isRoadCCW);
                    if (!evaluation.isAccepted()) continue;

                    double score = scoreOf(evaluation.angleDeviation(), evaluation.entranceLength());
                    if (bestScore <= score) continue;

                    bestScore = score;
                    bestPlan = evaluation.plan();
                }
            }
        }

        return bestPlan;
    }

    private Evaluation evaluateCandidate(
            TemporaryBuilding building, CandidatePair pair, boolean isBuildingCCW, boolean isRoadCCW) {
        DirectedEdge buildingEdge = pair.buildingEdge();
        DirectedEdge roadEdge = pair.roadEdge();
        TemporaryRoad road = pair.road();

        if (1 < map.getAttachedObjects(roadEdge).size()) {
            return Evaluation.rejected(pair, RejectReason.ROAD_EDGE_OCCUPIED);
        }
        if (roadEdge.getLength() < entranceWidth) {
            return Evaluation.rejected(pair, RejectReason.ROAD_EDGE_TOO_SHORT);
        }

        Point2D wallMidPoint = buildingEdge.getMidpoint();
        Point2D reachedPoint = findConnectingPoint(wallMidPoint, buildingEdge, isBuildingCCW, roadEdge.getLine());
        if (reachedPoint == null) {
            return Evaluation.rejected(pair, RejectReason.NO_CONNECTING_POINT);
        }
        if (pointsInward(reachedPoint.minus(wallMidPoint), buildingEdge, isBuildingCCW)) {
            return Evaluation.rejected(pair, RejectReason.ROAD_BEHIND_WALL);
        }
        if (pointsInward(wallMidPoint.minus(reachedPoint), roadEdge, isRoadCCW)) {
            return Evaluation.rejected(pair, RejectReason.WALL_BEHIND_ROAD_EDGE);
        }

        // Slide the connecting point so that the entrance fits within the road edge
        Point2D connectingPoint = slideIntoEdge(reachedPoint, roadEdge);
        Line2D entranceCentreLine = new Line2D(wallMidPoint, connectingPoint);
        double entranceLength = entranceCentreLine.getDirection().getLength();
        double angleDeviation = calculateAngleDeviation(entranceCentreLine, buildingEdge, roadEdge);

        Vector2D wallVector = buildingEdge.getLine().getDirection().normalised();
        Vector2D roadVector = roadEdge.getLine().getDirection().normalised();
        double halfWidth = entranceWidth / 2.0;
        Node b1 = map.getNode(wallMidPoint.plus(wallVector.scale(-halfWidth)));
        Node b2 = map.getNode(wallMidPoint.plus(wallVector.scale(halfWidth)));
        Node r1 = map.getNode(connectingPoint.plus(roadVector.scale(-halfWidth)));
        Node r2 = map.getNode(connectingPoint.plus(roadVector.scale(halfWidth)));

        // Build entrance edges, merging nearby nodes and skipping degenerate shapes.
        List<DirectedEdge> entranceEdges = buildEntranceEdges(b1, b2, r1, r2, wallVector, roadVector);
        if (entranceEdges == null) {
            return Evaluation.rejected(pair, RejectReason.DEGENERATE_ENTRANCE, entranceLength, angleDeviation);
        }
        if (connectingEdgesCrossOwnGeometry(entranceEdges, b1, b2, buildingEdge, roadEdge, building, road)) {
            return Evaluation.rejected(pair, RejectReason.CROSSES_OWN_GEOMETRY, entranceLength, angleDeviation);
        }

        TemporaryIntersection entrance = new TemporaryIntersection(entranceEdges);

        if (entranceLength < minConnectDistance) {
            return Evaluation.rejected(pair, RejectReason.TOO_SHORT, entranceLength, angleDeviation);
        }
        if (maxConnectDistance < entranceLength) {
            return Evaluation.rejected(pair, RejectReason.TOO_LONG, entranceLength, angleDeviation);
        }
        if (!isAngleAcceptable(angleDeviation)) {
            return Evaluation.rejected(pair, RejectReason.ANGLE_DEVIATION, entranceLength, angleDeviation);
        }

        Optional<TemporaryObject> collideWith = findCollidingObject(entrance, building, road);
        if (collideWith.isPresent()) {
            return Evaluation.collided(pair, entranceLength, angleDeviation, collideWith.get());
        }

        EntrancePlan plan = new EntrancePlan(entrance, buildingEdge.getEdge(), roadEdge.getEdge(), b1, b2, r1, r2);
        return Evaluation.accepted(pair, plan, entranceLength, angleDeviation);
    }

    // Returns the unit normal of the edge that points out of the polygon
    private Vector2D outwardNormal(DirectedEdge polygonEdge, boolean isCCW) {
        Vector2D edgeDirection = polygonEdge.getLine().getDirection().normalised();
        return isCCW ? edgeDirection.getNormal().negate() : edgeDirection.getNormal();
    }

    private boolean pointsInward(final Vector2D direction, final DirectedEdge polygonEdge, boolean isCCW) {
        return direction.dot(outwardNormal(polygonEdge, isCCW)) < 0;
    }

    private Point2D slideIntoEdge(Point2D point, DirectedEdge edge) {
        Line2D line = edge.getLine();
        Vector2D direction = line.getDirection().normalised();
        double halfWidth = entranceWidth / 2.0;
        double distFromStart = GeometryTools2D.getDistance(line.getOrigin(), point);
        double distFromEnd = edge.getLength() - distFromStart;

        if (distFromStart < halfWidth) return point.plus(direction.scale(halfWidth - distFromStart));
        if (distFromEnd < halfWidth) return point.plus(direction.scale(distFromEnd - halfWidth));
        return point;
    }

    // Returns the first object other than the building and road that overlaps the candidate
    private Optional<TemporaryObject> findCollidingObject(
            TemporaryIntersection candidate, TemporaryBuilding building, TemporaryRoad road) {
        Area entranceArea = new Area(candidate.getShape());
        return map.getAllObjects().stream()
                .filter(other -> !other.equals(building) && !other.equals(road))
                .filter(other -> overlaps(entranceArea, other))
                .findFirst();
    }

    private boolean overlaps(Area area, TemporaryObject object) {
        Area otherArea = new Area(object.getShape());
        otherArea.intersect(area);
        return !otherArea.isEmpty();
    }

    private boolean connectingEdgesCrossOwnGeometry(
            List<DirectedEdge> entranceEdges, final Node b1, final Node b2,
            DirectedEdge buildingEdge, final DirectedEdge roadEdge,
            TemporaryBuilding building, final TemporaryRoad road) {
        for (DirectedEdge entranceEdge : entranceEdges) {
            if (isWallOrRoadEdge(entranceEdge, b1, b2)) continue;

            if (crossAnyEdgeExcept(entranceEdge, building.getEdges(), buildingEdge)) return true;
            if (crossAnyEdgeExcept(entranceEdge, road.getEdges(), roadEdge)) return true;
        }
        return false;
    }

    private boolean isWallOrRoadEdge(final DirectedEdge edge, final Node b1, final Node b2) {
        boolean startOnBuildingSide = edge.getStartNode().equals(b1) || edge.getStartNode().equals(b2);
        boolean endOnBuildingSide = edge.getEndNode().equals(b1) || edge.getEndNode().equals(b2);
        return startOnBuildingSide == endOnBuildingSide;
    }

    private boolean crossAnyEdgeExcept(
            DirectedEdge candidate, List<DirectedEdge> edges, DirectedEdge excluded) {
        Line2D candidateLine = candidate.getLine();
        for (DirectedEdge edge : edges) {
            if (edge.equals(excluded)) continue;
            if (GeometryTools2D.getSegmentIntersectionPoint(candidateLine, edge.getLine()) != null) {
                return true;
            }
        }
        return false;
    }

    // Build a list of directed edges forming the entrance polygon from four corner nodes.
    // Nearby nodes that snap to the same position are deduplicated, yielding a triangle
    // when two corners coincide. Returns null if fewer than 3 distinct corners remain.
    private List<DirectedEdge> buildEntranceEdges(
            Node b1, Node b2, Node r1, Node r2, Vector2D wallVector, Vector2D roadVector) {
        // Preserve winding order based on the relative orientation of wall and road.
        List<Node> orderedCorners = 0 < wallVector.dot(roadVector)
                ? List.of(b1, b2, r2, r1)
                : List.of(b1, b2, r1, r2);
        List<Node> corners = new ArrayList<>(new LinkedHashSet<>(orderedCorners));

        // Skip if fewer than 3 distinct corners exist;
        // the entrance would not form a valid polygon.
        if (corners.size() < 3) return null;

        // Create entrance shape
        List<DirectedEdge> entranceEdges = new ArrayList<>();
        for (int j = 0; j < corners.size(); j++) {
            entranceEdges.add(map.getDirectedEdge(
                    corners.get(j),
                    corners.get((j + 1) % corners.size())));
        }

        return entranceEdges;
    }

    private double calculateAngleDeviation(Line2D centerLine, DirectedEdge buildingEdge, DirectedEdge roadEdge) {
        double angleToBuilding = Math.abs(90.0 - GeometryTools2D.getAngleBetweenVectors(
                centerLine.getDirection(), buildingEdge.getLine().getDirection()));
        double angleToRoad = Math.abs(90.0 - GeometryTools2D.getAngleBetweenVectors(
                centerLine.getDirection(), roadEdge.getLine().getDirection()));
        return Math.max(angleToBuilding, angleToRoad);
    }

    // Finds the point on a road edge where an entrance from the given wall would attach.
    // Returns null if the strategy cannot reach this road edge from the wall.
    private Point2D findConnectingPoint(
            Point2D wallMidPoint, DirectedEdge buildingEdge, boolean isBuildingCCW, Line2D roadLine) {
        return switch (strategy) {
            case MIN_ANGLE_DEVIATION -> GeometryTools2D.getClosestPointOnSegment(roadLine, wallMidPoint);
            case NEAREST_PERPENDICULAR -> {
                // Drop a perpendicular from the wall midpoint and find where it hits this road edge.
                Vector2D rayDirection = outwardNormal(buildingEdge, isBuildingCCW).scale(maxConnectDistance);
                Line2D perpendicular = new Line2D(wallMidPoint, wallMidPoint.plus(rayDirection));
                yield GeometryTools2D.getSegmentIntersectionPoint(perpendicular, roadLine);
            }
        };
    }

    // Only the angle-based strategy discards candidates by angle deviation.
    private boolean isAngleAcceptable(double angleDeviation) {
        return strategy != EntranceStrategy.MIN_ANGLE_DEVIATION || angleDeviation <= maxAngleDeviation;
    }

    // Returns the score used to rank candidates; lower is better.
    private double scoreOf(double angleDeviation, double entranceLength) {
        return switch (strategy) {
            case MIN_ANGLE_DEVIATION -> angleDeviation;
            case NEAREST_PERPENDICULAR -> entranceLength;
        };
    }

    private void visualizeResults(List<TemporaryIntersection> entrances) {
        StepVisualizer.create(debug)
                .title("Create Entrances")
                .layer(PolygonLayer.of(entrances)
                        .name("Entrances")
                        .outlineColor(DebugPalette.MOSS_STROKE)
                        .fillColor(DebugPalette.MOSS_FILL))
                .backgroundLayer(PolygonLayer.of(map.getAllObjects())
                        .name("Objects")
                        .outlineColor(DebugPalette.SLATE_STROKE)
                        .fillColor(DebugPalette.SLATE_FILL))
                .show();
    }

    private void recordResult(
            TemporaryBuilding building, EntranceResult result, Map<EntranceResult, Integer> counts) {
        counts.merge(result, 1, Integer::sum);
        LOGGER.info("ENTRANCE_RESULT building_id={} status={}", building.getId(), result);
    }

    private void logSummary(int total, Map<EntranceResult, Integer> counts) {
        LOGGER.info("ENTRANCE_SUMMARY strategy={} total={} already_connected={} connected={} not_connected={}",
                strategy, total,
                counts.getOrDefault(EntranceResult.ALREADY_CONNECTED, 0),
                counts.getOrDefault(EntranceResult.CONNECTED, 0),
                counts.getOrDefault(EntranceResult.NOT_CONNECTED, 0));
    }
}
