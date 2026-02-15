package frc.robot.pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* (Anytime Dynamic A*) running locally in a background thread.
 *
 * <p>Loads the navigation grid from the deploy directory and applies inflation for robot safety.
 *
 * <p>The algorithm runs in a background daemon thread, continuously updating the path as the start
 * or goal positions change.
 */
public class LocalADStar implements Pathfinder {
  // AD* algorithm constant (1.0 for standard A* behavior)
  private static final double EPS = 1.0;

  // Grid configuration
  private double m_nodeSize = PathfindingConstants.NODE_SIZE_METERS;
  private double m_fieldLength = PathfindingConstants.FIELD_LENGTH_METERS;
  private double m_fieldWidth = PathfindingConstants.FIELD_WIDTH_METERS;
  // Calculate grid dimensions
  private int m_nodesX = (int) Math.ceil(m_fieldLength / m_nodeSize);
  private int m_nodesY = (int) Math.ceil(m_fieldWidth / m_nodeSize);

  // AD* data structures
  private final HashMap<GridPosition, Double> m_g = new HashMap<>();
  private final HashMap<GridPosition, Double> m_rhs = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> m_open = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> m_incons = new HashMap<>();
  private final Set<GridPosition> m_closed = new HashSet<>();

  // Obstacle set (navgrid.json already includes robot buffer - no additional
  // inflation needed)
  private final Set<GridPosition> m_obstacles = new HashSet<>();

  // Request state (protected by lock)
  private final ReadWriteLock m_requestLock = new ReentrantReadWriteLock();
  private GridPosition m_requestStart = new GridPosition(0, 0);
  private GridPosition m_requestGoal = new GridPosition(0, 0);
  private Pose2d m_requestRealStartPose = Pose2d.kZero;
  private Pose2d m_requestRealGoalPose = Pose2d.kZero;
  private Translation2d m_requestStartVelocity = Translation2d.kZero;
  private boolean m_requestMinor = false;
  private boolean m_requestReset = false;
  // Track if goal was adjusted from obstacle - if so, don't use realGoalPose for
  // final waypoint
  private boolean m_goalWasInObstacle = false;

  // Path output (thread-safe)
  private final AtomicReference<List<Translation2d>> m_currentPathWaypoints =
      new AtomicReference<>(new ArrayList<>());
  private final AtomicReference<Pose2d> m_currentGoalPose = new AtomicReference<>(null);
  private final AtomicBoolean m_newPathAvailable = new AtomicBoolean(false);
  private final AtomicBoolean m_isComputing = new AtomicBoolean(false);

  // Background thread
  private final Thread m_planningThread;
  private volatile boolean m_running = true;

  /** Create a new pathfinder that runs AD* locally in a background thread. */
  public LocalADStar() {
    // Load obstacles from navgrid.json (already includes robot buffer from
    // PathPlanner)
    loadObstacles("pathplanner/navgrid.json");

    System.out.println("[LocalADStar] Grid: " + m_nodesX + "x" + m_nodesY + " nodes");
    System.out.println("[LocalADStar] Node size: " + m_nodeSize + "m");
    System.out.println(
        "[LocalADStar] Obstacles: " + m_obstacles.size() + " cells (from navgrid with buffer)");

    // Start background planning thread
    m_planningThread = new Thread(this::runThread);
    m_planningThread.setDaemon(true);
    m_planningThread.setName("ADStar Planning Thread");
    m_planningThread.start();
  }

  /**
   * Load obstacles from a navgrid JSON file.
   *
   * @param path Path relative to deploy directory
   */
  private void loadObstacles(String path) {
    File navGridFile = new File(Filesystem.getDeployDirectory(), path);
    if (!navGridFile.exists()) {
      System.err.println("[LocalADStar] Unable to load navgrid: " + path);
      return;
    }

    try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
      var fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      var json = (JSONObject) new JSONParser().parse(fileContent);

      // Read grid configuration
      m_nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();

      var fieldSize = (JSONObject) json.get("field_size");
      m_fieldLength = ((Number) fieldSize.get("x")).doubleValue();
      m_fieldWidth = ((Number) fieldSize.get("y")).doubleValue();

      // Read obstacle grid
      var grid = (JSONArray) json.get("grid");
      m_nodesY = grid.size();

      for (int row = 0; row < grid.size(); row++) {
        var rowArray = (JSONArray) grid.get(row);
        if (row == 0) {
          m_nodesX = rowArray.size();
        }
        for (int col = 0; col < rowArray.size(); col++) {
          boolean isObstacle = (boolean) rowArray.get(col);
          if (isObstacle) {
            m_obstacles.add(new GridPosition(col, row));
          }
        }
      }

      System.out.println("[LocalADStar] Successfully loaded navgrid: " + path);
    } catch (Exception e) {
      System.err.println("[LocalADStar] Error loading navgrid: " + e.getMessage());
      e.printStackTrace();
    }
  }

  // Note: No inflation needed - PathPlanner's navgrid.json already includes robot
  // buffer
  // ==================== Pathfinder Interface ====================

  @Override
  public boolean isNewPathAvailable() {
    return m_newPathAvailable.get();
  }

  @Override
  public List<Translation2d> getCurrentPathWaypoints() {
    m_newPathAvailable.set(false);
    return m_currentPathWaypoints.get();
  }

  @Override
  public Pose2d getGoalPose() {
    return m_currentGoalPose.get();
  }

  @Override
  public void setStartPose(Pose2d startPose) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPose.getTranslation()));
    if (startPos != null && !startPos.equals(m_requestStart)) {
      m_requestLock.writeLock().lock();
      try {
        m_requestStart = startPos;
        m_requestRealStartPose = startPose;
        m_requestMinor = true;
        m_newPathAvailable.set(false);
      } finally {
        m_requestLock.writeLock().unlock();
      }
    }
  }

  @Override
  public void setGoalPose(Pose2d goalPose) {
    GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPose.getTranslation()));
    if (gridPos != null) {
      m_requestLock.writeLock().lock();
      try {
        m_requestGoal = gridPos;
        m_requestRealGoalPose = goalPose;
        m_requestMinor = true;
        m_requestReset = true;
        m_newPathAvailable.set(false);
        m_currentGoalPose.set(goalPose);
      } finally {
        m_requestLock.writeLock().unlock();
      }
    }
  }

  @Override
  public void setStartVelocity(Translation2d startVelocity) {
    m_requestLock.writeLock().lock();
    try {
      m_requestStartVelocity = startVelocity;
    } finally {
      m_requestLock.writeLock().unlock();
    }
  }

  @Override
  public void setProblem(Pose2d startPose, Pose2d goalPose, Translation2d startVelocity) {
    m_requestLock.writeLock().lock();
    try {
      // === DEBUG: Raw input poses ===
      System.out.println("\n========== AD* SET PROBLEM DEBUG ==========");
      System.out.println(
          "[AD*] OBSTACLE CHECK: " + m_obstacles.size() + " obstacles (from navgrid with buffer)");
      System.out.println("[AD*] Grid dimensions: " + m_nodesX + "x" + m_nodesY);
      System.out.println("[AD*] START Pose: (" + String.format("%.3f", startPose.getX()) + ", "
          + String.format("%.3f", startPose.getY()) + ", "
          + String.format("%.1f", startPose.getRotation().getDegrees()) + "°)");
      System.out.println("[AD*] GOAL Pose: (" + String.format("%.3f", goalPose.getX()) + ", "
          + String.format("%.3f", goalPose.getY()) + ", "
          + String.format("%.1f", goalPose.getRotation().getDegrees()) + "°)");
      System.out.println("[AD*] Start Velocity: (" + String.format("%.3f", startVelocity.getX())
          + ", " + String.format("%.3f", startVelocity.getY()) + ") m/s");

      GridPosition rawStartGrid = getGridPos(startPose.getTranslation());
      GridPosition rawGoalGrid = getGridPos(goalPose.getTranslation());
      System.out.println(
          "[AD*] Raw Start Grid: (" + rawStartGrid.x() + ", " + rawStartGrid.y() + ")");
      System.out.println("[AD*] Raw Goal Grid: (" + rawGoalGrid.x() + ", " + rawGoalGrid.y() + ")");
      System.out.println("[AD*] Start in obstacle? " + m_obstacles.contains(rawStartGrid));
      System.out.println("[AD*] Goal in obstacle? " + m_obstacles.contains(rawGoalGrid));

      // Track if goal is in an obstacle - if so, we'll use the adjusted position for
      // final waypoint
      m_goalWasInObstacle = m_obstacles.contains(rawGoalGrid);

      GridPosition startPos = findClosestNonObstacle(getGridPos(startPose.getTranslation()));
      if (startPos != null) {
        m_requestStart = startPos;
        m_requestRealStartPose = startPose;
        System.out.println(
            "[AD*] Adjusted Start Grid: (" + startPos.x() + ", " + startPos.y() + ")");
        System.out.println("[AD*] Start Grid → Field: ("
            + String.format("%.3f", gridPosToTranslation(startPos).getX()) + ", "
            + String.format("%.3f", gridPosToTranslation(startPos).getY()) + ")");
      } else {
        System.out.println("[AD*] ERROR: Could not find non-obstacle start position!");
      }
      GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPose.getTranslation()));
      if (gridPos != null) {
        m_requestGoal = gridPos;
        m_requestRealGoalPose = goalPose;
        m_currentGoalPose.set(goalPose);
        System.out.println("[AD*] Adjusted Goal Grid: (" + gridPos.x() + ", " + gridPos.y() + ")");
        System.out.println("[AD*] Goal Grid → Field: ("
            + String.format("%.3f", gridPosToTranslation(gridPos).getX()) + ", "
            + String.format("%.3f", gridPosToTranslation(gridPos).getY()) + ")");
      } else {
        System.out.println("[AD*] ERROR: Could not find non-obstacle goal position!");
      }
      m_requestStartVelocity = startVelocity;
      m_requestMinor = true;
      m_requestReset = true;
      m_newPathAvailable.set(false);
      System.out.println("=========================================\n");
    } finally {
      m_requestLock.writeLock().unlock();
    }
  }

  @Override
  public void setTeleopObstacles() {
    // Currently only using one obstacle set
    System.out.println("[LocalADStar] Using teleop obstacles");
  }

  @Override
  public void setAutoObstacles() {
    // Currently only using one obstacle set
    System.out.println("[LocalADStar] Using auto obstacles");
  }

  @Override
  public boolean isComputing() {
    return m_isComputing.get();
  }

  @Override
  public void shutdown() {
    m_running = false;
    m_planningThread.interrupt();
  }

  // ==================== Background Thread ====================

  @SuppressWarnings("BusyWait")
  private void runThread() {
    System.out.println("[AD*] Background planning thread started!");

    while (m_running) {
      try {
        // Use WRITE lock to safely read AND modify flags
        m_requestLock.writeLock().lock();
        boolean reset = m_requestReset;
        boolean minor = m_requestMinor;

        // Clear the flags now that we've read them
        if (reset) {
          m_requestReset = false;
        }
        if (minor) {
          m_requestMinor = false;
        }

        GridPosition start = m_requestStart;
        GridPosition goal = m_requestGoal;
        Pose2d realStart = m_requestRealStartPose;
        Pose2d realGoal = m_requestRealGoalPose;
        m_requestLock.writeLock().unlock();

        if (reset || minor) {
          System.out.println("[AD*] Thread processing: reset=" + reset + ", minor=" + minor);
          m_isComputing.set(true);
          doWork(reset, minor, start, goal, realStart, realGoal);
          m_isComputing.set(false);
          System.out.println("[AD*] Thread finished computing path");
        } else {
          Thread.sleep(10);
        }
      } catch (InterruptedException e) {
        // Thread interrupted, check running flag
      } catch (Exception e) {
        System.err.println("[LocalADStar] Error in planning thread: " + e.getMessage());
        e.printStackTrace();
        // Reset and continue
        m_requestLock.writeLock().lock();
        try {
          m_requestReset = true;
        } finally {
          m_requestLock.writeLock().unlock();
        }
      }
    }
  }

  private void doWork(
      boolean needsReset,
      boolean doMinor,
      GridPosition sStart,
      GridPosition sGoal,
      Pose2d realStartPose,
      Pose2d realGoalPose) {

    if (needsReset) {
      System.out.println("[AD*] Resetting algorithm...");
      reset(sStart, sGoal);
    }

    if (doMinor) {
      System.out.println("\n========== AD* PATH COMPUTATION DEBUG ==========");
      System.out.println("[AD*] Computing path from grid (" + sStart.x() + ", " + sStart.y()
          + ") to (" + sGoal.x() + ", " + sGoal.y() + ")");

      computeOrImprovePath(sStart, sGoal);

      List<GridPosition> pathPositions = extractPath(sStart, sGoal);
      System.out.println("[AD*] Extracted " + pathPositions.size() + " grid positions");

      // Log raw grid path
      if (pathPositions.size() > 0) {
        System.out.println("[AD*] Grid Path:");
        for (int i = 0; i < Math.min(pathPositions.size(), 10); i++) {
          GridPosition gp = pathPositions.get(i);
          Translation2d fieldPos = gridPosToTranslation(gp);
          System.out.println("  [" + i + "] Grid(" + gp.x() + ", " + gp.y() + ") → Field("
              + String.format("%.2f", fieldPos.getX()) + ", "
              + String.format("%.2f", fieldPos.getY()) + ")");
        }
        if (pathPositions.size() > 10) {
          System.out.println("  ... (" + (pathPositions.size() - 10) + " more positions)");
        }
      }

      List<Translation2d> waypoints =
          convertToWaypoints(pathPositions, realStartPose, realGoalPose);

      // Log final waypoints
      System.out.println("[AD*] Converted to " + waypoints.size() + " waypoints:");
      for (int i = 0; i < waypoints.size(); i++) {
        Translation2d wp = waypoints.get(i);
        System.out.println("  WP[" + i + "]: (" + String.format("%.3f", wp.getX()) + ", "
            + String.format("%.3f", wp.getY()) + ")");
      }
      System.out.println("================================================\n");

      // === PUBLISH TO SMARTDASHBOARD FOR ADVANTAGESCOPE ===
      SmartDashboard.putNumber("Pathfinding/AD*/Obstacles", m_obstacles.size());
      SmartDashboard.putNumber("Pathfinding/AD*/GridPathLength", pathPositions.size());
      SmartDashboard.putNumber("Pathfinding/AD*/FinalWaypointCount", waypoints.size());
      SmartDashboard.putBoolean("Pathfinding/AD*/PathFound", waypoints.size() >= 2);

      m_currentPathWaypoints.set(waypoints);
      m_newPathAvailable.set(true);
    }
  }

  // ==================== AD* Algorithm ====================

  private void reset(GridPosition sStart, GridPosition sGoal) {
    m_g.clear();
    m_rhs.clear();
    m_open.clear();
    m_incons.clear();
    m_closed.clear();

    // Initialize all nodes with infinite cost
    for (int x = 0; x < m_nodesX; x++) {
      for (int y = 0; y < m_nodesY; y++) {
        GridPosition pos = new GridPosition(x, y);
        m_g.put(pos, Double.POSITIVE_INFINITY);
        m_rhs.put(pos, Double.POSITIVE_INFINITY);
      }
    }

    // Goal has zero cost-to-go
    m_rhs.put(sGoal, 0.0);
    m_open.put(sGoal, key(sGoal, sStart));
  }

  private void computeOrImprovePath(GridPosition sStart, GridPosition sGoal) {
    int iterations = 0;
    int maxIterations = m_nodesX * m_nodesY * 2; // Safety limit

    while (iterations++ < maxIterations) {
      var sv = topKey();
      if (sv == null) {
        break;
      }

      GridPosition s = sv.getFirst();
      Pair<Double, Double> v = sv.getSecond();

      if (comparePair(v, key(sStart, sStart)) >= 0
          && m_rhs
              .getOrDefault(sStart, Double.POSITIVE_INFINITY)
              .equals(m_g.getOrDefault(sStart, Double.POSITIVE_INFINITY))) {
        break;
      }

      m_open.remove(s);

      if (m_g.getOrDefault(s, Double.POSITIVE_INFINITY)
          > m_rhs.getOrDefault(s, Double.POSITIVE_INFINITY)) {
        m_g.put(s, m_rhs.get(s));
        m_closed.add(s);

        for (GridPosition sn : getOpenNeighbors(s)) {
          updateState(sn, sStart, sGoal);
        }
      } else {
        m_g.put(s, Double.POSITIVE_INFINITY);
        for (GridPosition sn : getOpenNeighbors(s)) {
          updateState(sn, sStart, sGoal);
        }
        updateState(s, sStart, sGoal);
      }
    }
  }

  private void updateState(GridPosition s, GridPosition sStart, GridPosition sGoal) {
    if (!s.equals(sGoal)) {
      double minRhs = Double.POSITIVE_INFINITY;
      for (GridPosition x : getOpenNeighbors(s)) {
        double cost = m_g.getOrDefault(x, Double.POSITIVE_INFINITY) + cost(s, x);
        if (cost < minRhs) {
          minRhs = cost;
        }
      }
      m_rhs.put(s, minRhs);
    }

    m_open.remove(s);

    if (!m_g.getOrDefault(s, Double.POSITIVE_INFINITY)
        .equals(m_rhs.getOrDefault(s, Double.POSITIVE_INFINITY))) {
      if (!m_closed.contains(s)) {
        m_open.put(s, key(s, sStart));
      } else {
        m_incons.put(s, Pair.of(0.0, 0.0));
      }
    }
  }

  private List<GridPosition> extractPath(GridPosition sStart, GridPosition sGoal) {
    if (sGoal.equals(sStart)) {
      return List.of(sStart, sGoal); // Need at least 2 points
    }

    List<GridPosition> path = new ArrayList<>();
    path.add(sStart);

    GridPosition s = sStart;
    for (int k = 0; k < 400; k++) {
      List<GridPosition> neighbors = getOpenNeighbors(s);

      // Debug: log neighbor count on first iteration
      if (k == 0) {
        System.out.println("[AD*] extractPath: Start (" + s.x() + ", " + s.y() + ") has "
            + neighbors.size() + " open neighbors");
        if (neighbors.isEmpty()) {
          System.out.println(
              "[AD*] ERROR: No open neighbors from start! Checking raw obstacles...");
          System.out.println("[AD*] Start in raw obstacles? " + m_obstacles.contains(s));
        }
      }

      // Find neighbor with minimum g-cost (closest to goal)
      GridPosition best = null;
      double bestCost = Double.POSITIVE_INFINITY;

      for (GridPosition x : neighbors) {
        double gCost = m_g.getOrDefault(x, Double.POSITIVE_INFINITY);
        if (gCost < bestCost) {
          bestCost = gCost;
          best = x;
        }
      }

      // If no valid neighbor found, fall back to direct path if possible
      if (best == null) {
        System.out.println("[AD*] extractPath: No neighbor with finite g-cost at step " + k);
        // Try to reach goal directly if we're stuck
        if (path.size() == 1) {
          System.out.println("[AD*] Adding goal directly since path extraction failed");
          path.add(sGoal);
        }
        break;
      }

      s = best;
      path.add(s);

      if (s.equals(sGoal)) {
        break;
      }
    }

    return path;
  }

  private List<Translation2d> convertToWaypoints(
      List<GridPosition> path, Pose2d realStartPose, Pose2d realGoalPose) {

    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    // Simplify path using line-of-sight
    List<GridPosition> simplifiedPath = simplifyPath(path);

    // Convert to field coordinates
    List<Translation2d> waypoints = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      waypoints.add(gridPosToTranslation(pos));
    }

    // Replace start with actual pose (always safe to use real start position)
    // Only replace end if goal was NOT in an obstacle - otherwise keep the adjusted
    // safe position
    if (!waypoints.isEmpty()) {
      waypoints.set(0, realStartPose.getTranslation());
      if (!m_goalWasInObstacle) {
        // Goal was in free space, use exact requested position
        waypoints.set(waypoints.size() - 1, realGoalPose.getTranslation());
      } else {
        // Goal was in obstacle - keep the grid-adjusted safe position (already set from
        // loop above)
        System.out.println("[AD*] Goal was in obstacle - using adjusted safe position instead of "
            + "original goal (" + String.format("%.3f", realGoalPose.getX()) + ", "
            + String.format("%.3f", realGoalPose.getY()) + ")");
      }
    }

    return waypoints;
  }

  private List<GridPosition> simplifyPath(List<GridPosition> path) {
    if (path.size() <= 2) {
      return path;
    }

    List<GridPosition> simplified = new ArrayList<>();
    simplified.add(path.get(0));

    int current = 0;
    while (current < path.size() - 1) {
      int farthest = current + 1;

      // Find the farthest visible point
      for (int i = current + 2; i < path.size(); i++) {
        if (walkable(path.get(current), path.get(i))) {
          farthest = i;
        }
      }

      simplified.add(path.get(farthest));
      current = farthest;
    }

    return simplified;
  }

  // ==================== Helper Methods ====================

  private double cost(GridPosition s1, GridPosition s2) {
    if (isCollision(s1, s2)) {
      return Double.POSITIVE_INFINITY;
    }
    return heuristic(s1, s2);
  }

  private boolean isCollision(GridPosition s1, GridPosition s2) {
    // Check against raw obstacles for collision (inflated used for
    // findClosestNonObstacle)
    if (m_obstacles.contains(s1) || m_obstacles.contains(s2)) {
      return true;
    }

    // Check diagonal corners
    if (s1.x() != s2.x() && s1.y() != s2.y()) {
      GridPosition corner1 = new GridPosition(s1.x(), s2.y());
      GridPosition corner2 = new GridPosition(s2.x(), s1.y());
      return m_obstacles.contains(corner1) || m_obstacles.contains(corner2);
    }

    return false;
  }

  private boolean walkable(GridPosition s1, GridPosition s2) {
    // Bresenham's line algorithm
    int x0 = s1.x();
    int y0 = s1.y();
    int x1 = s2.x();
    int y1 = s2.y();

    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int xInc = (x1 > x0) ? 1 : -1;
    int yInc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; n--) {
      // Use raw obstacles for walkability check
      if (m_obstacles.contains(new GridPosition(x, y))) {
        return false;
      }

      if (error > 0) {
        x += xInc;
        error -= dy;
      } else if (error < 0) {
        y += yInc;
        error += dx;
      } else {
        x += xInc;
        y += yInc;
        error -= dy;
        error += dx;
        n--;
      }
    }

    return true;
  }

  private List<GridPosition> getOpenNeighbors(GridPosition s) {
    List<GridPosition> neighbors = new ArrayList<>();

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        int nx = s.x() + dx;
        int ny = s.y() + dy;

        if (nx >= 0 && nx < m_nodesX && ny >= 0 && ny < m_nodesY) {
          GridPosition neighbor = new GridPosition(nx, ny);
          // Only check raw obstacles, not inflated - this allows the algorithm to find
          // paths
          // The inflation is for safety, but the robot's actual path can go near
          // obstacles
          if (!m_obstacles.contains(neighbor)) {
            neighbors.add(neighbor);
          }
        }
      }
    }

    return neighbors;
  }

  private GridPosition findClosestNonObstacle(GridPosition pos) {
    // First, ensure the starting position is clamped to valid grid bounds
    int clampedX = Math.max(0, Math.min(pos.x(), m_nodesX - 1));
    int clampedY = Math.max(0, Math.min(pos.y(), m_nodesY - 1));
    var clampedPos = new GridPosition(clampedX, clampedY);

    // NavGrid already includes robot buffer, so we check against obstacles
    // directly
    if (!m_obstacles.contains(clampedPos)) {
      return clampedPos;
    }

    // BFS to find nearest non-obstacle with a maximum search radius
    Set<GridPosition> visited = new HashSet<>();
    Queue<GridPosition> queue = new LinkedList<>();
    queue.add(clampedPos);

    int maxSearchIterations = m_nodesX * m_nodesY; // Limit to prevent infinite loops
    int iterations = 0;

    while (!queue.isEmpty() && iterations < maxSearchIterations) {
      iterations++;
      GridPosition check = queue.poll();

      // Check if this position is valid and not an obstacle
      if (check.x() >= 0
          && check.x() < m_nodesX
          && check.y() >= 0
          && check.y() < m_nodesY
          && !m_obstacles.contains(check)) {
        return check;
      }

      visited.add(check);

      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          int nx = check.x() + dx;
          int ny = check.y() + dy;
          // Only add neighbors that are within bounds
          if (nx >= 0 && nx < m_nodesX && ny >= 0 && ny < m_nodesY) {
            GridPosition neighbor = new GridPosition(nx, ny);
            if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
              queue.add(neighbor);
            }
          }
        }
      }
    }

    System.err.println("[AD*] WARNING: Could not find non-obstacle position near " + pos);
    return null;
  }

  private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
    double gVal = m_g.getOrDefault(s, Double.POSITIVE_INFINITY);
    double rhsVal = m_rhs.getOrDefault(s, Double.POSITIVE_INFINITY);

    if (gVal > rhsVal) {
      return Pair.of(rhsVal + EPS * heuristic(sStart, s), rhsVal);
    } else {
      return Pair.of(gVal + heuristic(sStart, s), gVal);
    }
  }

  private Pair<GridPosition, Pair<Double, Double>> topKey() {
    Map.Entry<GridPosition, Pair<Double, Double>> min = null;
    for (var entry : m_open.entrySet()) {
      if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
        min = entry;
      }
    }
    return min == null ? null : Pair.of(min.getKey(), min.getValue());
  }

  private double heuristic(GridPosition s1, GridPosition s2) {
    return Math.hypot(s2.x() - s1.x(), s2.y() - s1.y());
  }

  private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
    int first = Double.compare(a.getFirst(), b.getFirst());
    return first != 0 ? first : Double.compare(a.getSecond(), b.getSecond());
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.floor(pos.getX() / m_nodeSize);
    int y = (int) Math.floor(pos.getY() / m_nodeSize);
    // Clamp to valid grid bounds to handle edge cases (robot at/near origin or
    // outside field)
    x = Math.max(0, Math.min(x, m_nodesX - 1));
    y = Math.max(0, Math.min(y, m_nodesY - 1));
    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation(GridPosition pos) {
    return new Translation2d(
        (pos.x() * m_nodeSize) + (m_nodeSize / 2.0), (pos.y() * m_nodeSize) + (m_nodeSize / 2.0));
  }
}
