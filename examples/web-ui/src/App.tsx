import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import type { GaussianSource, LayerVisibility, Pose3D, SceneManifest } from "./lib/gs/gsSdfSceneAdapter";
import {
  createCurrentCameraPoseTopic,
  publishCurrentCameraPose,
  type GoalPose2D,
  type InitialPose2D
} from "./lib/ros/navGoalPublisher";
import type { DecodedRosImage } from "./lib/ros/rosImage";
import { useNeuralMappingRos, type PlaybackCloudOptions } from "./lib/ros/useNeuralMappingRos";
import { LiveGsViewer, type GsControlMode, type ViewerMode } from "./lib/viewer/liveGsViewer";
import type { PlanPoint2D } from "./lib/map/localPlanner";
import TopDownMap, {
  type MapTool,
  type Measurement2D,
  type ObstacleRect2D
} from "./components/TopDownMap";

type ConsoleMode = "playback" | "live" | "gs" | "compare";
type PlaybackCloudMode = "playback" | "occupancy" | "both";
type PlaybackCloudProfile = "fast" | "balanced" | "quality";

const defaultSceneUrl = "/scenes/fast-livo2-compressed-live/manifest.json";
const playbackRates = [1, 2, 4, 8, 16, 32] as const;
const playbackDecayOptions = [0, 5, 15, 30] as const;
const navPreviewSpeeds = [0.6, 1.0, 1.6] as const;
const playbackCloudProfiles: Record<
  PlaybackCloudProfile,
  Pick<
    PlaybackCloudOptions,
    "maxInputPoints" | "maxAccumulatedPoints" | "voxelSize" | "publishEveryFrames" | "minVoxelObservations"
  >
> = {
  fast: {
    maxInputPoints: 1600,
    maxAccumulatedPoints: 90_000,
    voxelSize: 0.34,
    publishEveryFrames: 1,
    minVoxelObservations: 3
  },
  balanced: {
    maxInputPoints: 2200,
    maxAccumulatedPoints: 140_000,
    voxelSize: 0.28,
    publishEveryFrames: 1,
    minVoxelObservations: 2
  },
  quality: {
    maxInputPoints: 3200,
    maxAccumulatedPoints: 220_000,
    voxelSize: 0.22,
    publishEveryFrames: 1,
    minVoxelObservations: 2
  }
};

interface PlaybackStatus {
  bagPath: string;
  containerName: string;
  image: string;
  rate: number;
  paused: boolean;
  loop: boolean;
  requestedOffsetSec: number;
  currentOffsetSec: number;
  durationSec: number;
  bagStartSec: number;
  bagEndSec: number;
  playbackAlive?: boolean;
}

export default function App() {
  const searchParams = useMemo(() => new URLSearchParams(window.location.search), []);
  const sceneUrl = useMemo(
    () => searchParams.get("scene") ?? defaultSceneUrl,
    [searchParams]
  );
  const initialConsoleMode = useMemo<ConsoleMode>(() => {
    const requested = searchParams.get("mode");
    if (requested === "playback" || requested === "gs" || requested === "compare") {
      return requested;
    }
    return "live";
  }, [searchParams]);
  const initialGsControlMode = useMemo<GsControlMode>(
    () => (searchParams.get("controls") === "fps" ? "fps" : "orbit"),
    [searchParams]
  );
  const initialGaussianVariant = useMemo(
    () => searchParams.get("gaussian") ?? "balanced",
    [searchParams]
  );
  const initialPlaybackCloudMode = useMemo<PlaybackCloudMode>(() => {
    const requested = searchParams.get("cloudMode");
    if (requested === "playback" || requested === "occupancy" || requested === "both") {
      return requested;
    }
    return "both";
  }, [searchParams]);
  const initialPlaybackCloudProfile = useMemo<PlaybackCloudProfile>(() => {
    const requested = searchParams.get("cloudProfile");
    if (requested === "fast" || requested === "balanced" || requested === "quality") {
      return requested;
    }
    return "fast";
  }, [searchParams]);
  const defaultWsUrl = useMemo(() => `ws://${window.location.hostname || "localhost"}:9090`, []);
  const defaultPlaybackApiUrl = useMemo(
    () => `http://${window.location.hostname || "localhost"}:8765`,
    []
  );

  const [wsUrl, setWsUrl] = useState(defaultWsUrl);
  const [playbackApiUrl, setPlaybackApiUrl] = useState(defaultPlaybackApiUrl);
  const [manifest, setManifest] = useState<SceneManifest | null>(null);
  const [consoleMode, setConsoleMode] = useState<ConsoleMode>(initialConsoleMode);
  const [gsControlMode, setGsControlMode] = useState<GsControlMode>(initialGsControlMode);
  const [gaussianVariant, setGaussianVariant] = useState<string | null>(initialGaussianVariant);
  const [playbackCloudMode, setPlaybackCloudMode] = useState<PlaybackCloudMode>(initialPlaybackCloudMode);
  const [playbackCloudProfile, setPlaybackCloudProfile] = useState<PlaybackCloudProfile>(
    initialPlaybackCloudProfile
  );
  const [playbackDecaySec, setPlaybackDecaySec] = useState<number>(15);
  const [goals, setGoals] = useState<GoalPose2D[]>([]);
  const [initialPose, setInitialPose] = useState<InitialPose2D | null>(null);
  const [plannedPath, setPlannedPath] = useState<PlanPoint2D[]>([]);
  const [plannerStatus, setPlannerStatus] = useState<"idle" | "ready" | "blocked">("idle");
  const [plannerMessage, setPlannerMessage] = useState<string | null>(null);
  const [measurements, setMeasurements] = useState<Measurement2D[]>([]);
  const [obstacles, setObstacles] = useState<ObstacleRect2D[]>([]);
  const [erasures, setErasures] = useState<ObstacleRect2D[]>([]);
  const [followRobot, setFollowRobot] = useState(false);
  const [mapTool, setMapTool] = useState<MapTool>("goal");
  const [mapExportRequestVersion, setMapExportRequestVersion] = useState(0);
  const [mapStatusMessage, setMapStatusMessage] = useState<string | null>(null);
  const [navPreviewActive, setNavPreviewActive] = useState(false);
  const [navPreviewSpeed, setNavPreviewSpeed] = useState<number>(1);
  const [navPreviewPose, setNavPreviewPose] = useState<Pose3D | null>(null);
  const [navPreviewTrajectory, setNavPreviewTrajectory] = useState<Pose3D[]>([]);
  const [navPreviewMessage, setNavPreviewMessage] = useState<string | null>(null);
  const [layers, setLayers] = useState<LayerVisibility>(() =>
    layerPresetForMode(initialConsoleMode)
  );
  const [sceneError, setSceneError] = useState<string | null>(null);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [playbackStatus, setPlaybackStatus] = useState<PlaybackStatus | null>(null);
  const [playbackBusy, setPlaybackBusy] = useState(false);
  const [playbackError, setPlaybackError] = useState<string | null>(null);
  const [playbackDraftOffset, setPlaybackDraftOffset] = useState<number | null>(null);
  const playbackStatusRequestInFlightRef = useRef(false);
  const playbackStatusFailureCountRef = useRef(0);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const viewerRef = useRef<LiveGsViewer | null>(null);

  const playbackCloudOptions = useMemo<PlaybackCloudOptions>(
    () => ({
      enabled: consoleMode !== "playback" ? true : layers.liveCloud,
      decayTimeSec: playbackDecaySec,
      ...playbackCloudProfiles[playbackCloudProfile]
    }),
    [consoleMode, layers.liveCloud, playbackCloudProfile, playbackDecaySec]
  );
  const rosState = useNeuralMappingRos(wsUrl, {
    playbackCloud: playbackCloudOptions
  });
  const effectiveManifest = useMemo(
    () => resolveGaussianVariant(manifest, gaussianVariant),
    [manifest, gaussianVariant]
  );
  const gaussianVariants = useMemo(
    () => getGaussianVariants(manifest),
    [manifest]
  );
  const activeGaussianVariant =
    effectiveManifest?.gaussian?.variant ??
    (manifest?.gaussianVariants?.balanced ? "balanced" : manifest?.gaussian?.variant ?? "balanced");
  const occupancyCloudAssetState = manifest?.assets?.rawPointCloud ?? "loading";
  const occupancyCloudLabel =
    consoleMode === "playback"
      ? "Accumulated Map"
      : occupancyCloudAssetState === "fallback"
        ? "Occupancy Cloud"
        : "Static Cloud";
  const liveCloudLabel = consoleMode === "playback" ? "Current Scan" : "ROS Cloud";
  const hasLiveCloud = Boolean(
    rosState.livePointCloud ||
      rosState.playbackAccumulatedPointCloud ||
      rosState.playbackScanPointCloud
  );
  const activeMapPointCloud =
    consoleMode === "playback"
      ? rosState.playbackAccumulatedPointCloud ?? rosState.livePointCloud
      : rosState.livePointCloud;
  const displayRobotPose = navPreviewPose ?? rosState.robotPose;
  const displayTrajectory = navPreviewActive ? navPreviewTrajectory : rosState.trajectory;

  const viewerMode: ViewerMode =
    consoleMode === "gs" || consoleMode === "compare"
      ? "gs"
      : consoleMode === "playback"
        ? "playback"
        : "live";
  const modeLabel =
    consoleMode === "compare"
      ? "Compare Review"
      : consoleMode === "gs"
        ? "GS Layer"
        : consoleMode === "playback"
          ? "Playback Mapping"
          : "Live Mapping";
  const mainViewSource =
    consoleMode === "playback"
      ? layers.liveCloud
        ? layers.occupancyCloud
          ? "Current scan + accumulated map"
          : "Current playback scan"
        : layers.occupancyCloud
          ? "Accumulated playback map"
          : "Playback geometry / overlays"
      : viewerMode === "gs"
      ? "Gaussian + mesh"
      : layers.occupancyCloud && occupancyCloudAssetState === "fallback"
        ? "Fallback occupancy cloud"
        : layers.liveCloud
          ? "ROS / playback cloud"
          : layers.occupancyCloud
            ? "Static cloud / occupancy cloud"
          : "Mesh / overlays";
  const workspaceSemantic = describeWorkspaceSemantic(
    consoleMode,
    viewerMode,
    occupancyCloudAssetState,
    rosState.streamProfile
  );
  const dataSemantic = describeDataSemantic(
    rosState.connectionState,
    rosState.streamProfile,
    consoleMode === "playback" ? rosState.playbackAccumulatedPointCloud ?? rosState.livePointCloud : rosState.livePointCloud,
    occupancyCloudAssetState
  );
  const renderSemantic = describeRenderSemantic(
    consoleMode,
    viewerMode,
    activeGaussianVariant,
    gsControlMode
  );
  const effectivePlaybackOffset = playbackDraftOffset ?? playbackStatus?.currentOffsetSec ?? 0;
  const playbackTimelineDisabled = !playbackStatus || playbackBusy;
  const playbackVideoStreamUrl = useMemo(
    () => buildPlaybackVideoUrl(playbackApiUrl, "/frame.mjpeg"),
    [playbackApiUrl]
  );
  const commitPlaybackDraft = () => {
    if (playbackDraftOffset === null || !playbackStatus) {
      return;
    }
    void sendPlaybackControl({
      rate: playbackStatus.rate,
      paused: playbackStatus.paused,
      offsetSec: playbackDraftOffset
    });
  };

  useEffect(() => {
    if (consoleMode !== "playback") {
      return;
    }

    let disposed = false;
    let hasReceivedStatus = false;
    const loadStatus = async () => {
      if (playbackStatusRequestInFlightRef.current) {
        return;
      }

      playbackStatusRequestInFlightRef.current = true;
      try {
        const response = await fetch(`${playbackApiUrl}/status`, { cache: "no-store" });
        if (!response.ok) {
          throw new Error(`Playback control returned ${response.status}`);
        }
        const status = (await response.json()) as PlaybackStatus;
        playbackStatusFailureCountRef.current = 0;
        hasReceivedStatus = true;
        if (!disposed) {
          setPlaybackStatus(status);
          setPlaybackError(null);
        }
      } catch (error) {
        playbackStatusFailureCountRef.current += 1;
        if (!disposed) {
          if (playbackStatusFailureCountRef.current >= 3) {
            setPlaybackError(error instanceof Error ? error.message : "Playback control unavailable.");
          }
        }
      } finally {
        playbackStatusRequestInFlightRef.current = false;
      }
    };

    const warmupTimerId = window.setTimeout(() => {
      if (!disposed && !hasReceivedStatus) {
        setPlaybackError("Playback control is still starting.");
      }
    }, 2000);

    void loadStatus();
    const intervalId = window.setInterval(loadStatus, 500);
    return () => {
      disposed = true;
      window.clearTimeout(warmupTimerId);
      window.clearInterval(intervalId);
      playbackStatusRequestInFlightRef.current = false;
      playbackStatusFailureCountRef.current = 0;
    };
  }, [consoleMode, playbackApiUrl]);

  const sendPlaybackControl = async (payload: {
    rate?: number;
    paused?: boolean;
    offsetSec?: number;
  }) => {
    setPlaybackBusy(true);
    try {
      const response = await fetch(`${playbackApiUrl}/control`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify(payload)
      });
      if (!response.ok) {
        throw new Error(`Playback control returned ${response.status}`);
      }
      const status = (await response.json()) as PlaybackStatus;
      setPlaybackStatus(status);
      setPlaybackDraftOffset(null);
      setPlaybackError(null);
    } catch (error) {
      setPlaybackError(error instanceof Error ? error.message : "Failed to update playback state.");
    } finally {
      setPlaybackBusy(false);
    }
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    try {
      const viewer = new LiveGsViewer(canvas);
      viewerRef.current = viewer;
      viewer.setMode(viewerMode);
      viewer.setGsControlMode(gsControlMode);
      viewer.setGaussianVariant(activeGaussianVariant);
      viewer.setVisibility(layers);
      setSceneError(null);

      return () => {
        viewer.dispose();
        viewerRef.current = null;
      };
    } catch (error) {
      setSceneError(error instanceof Error ? error.message : "Failed to initialize 3D viewer.");
      viewerRef.current = null;
      return;
    }
  }, []);

  useEffect(() => {
    viewerRef.current?.setMode(viewerMode);
  }, [viewerMode]);

  useEffect(() => {
    viewerRef.current?.setGsControlMode(gsControlMode);
  }, [gsControlMode]);

  useEffect(() => {
    viewerRef.current?.setGaussianVariant(activeGaussianVariant);
  }, [activeGaussianVariant]);

  useEffect(() => {
    viewerRef.current?.setVisibility(layers);
  }, [layers]);

  useEffect(() => {
    viewerRef.current?.setFollowRobot(followRobot);
  }, [followRobot]);

  useEffect(() => {
    viewerRef.current?.updateRobotPose(displayRobotPose);
  }, [displayRobotPose]);

  useEffect(() => {
    viewerRef.current?.updateTrajectory(displayTrajectory);
  }, [displayTrajectory]);

  useEffect(() => {
    viewerRef.current?.updatePlannedPath(plannedPath);
  }, [plannedPath]);

  useEffect(() => {
    viewerRef.current?.updateLivePointCloud(rosState.livePointCloud);
  }, [rosState.livePointCloud]);

  useEffect(() => {
    viewerRef.current?.updatePlaybackMapPointCloud(rosState.playbackAccumulatedPointCloud);
  }, [rosState.playbackAccumulatedPointCloud]);

  useEffect(() => {
    viewerRef.current?.updatePlaybackScanPointCloud(rosState.playbackScanPointCloud);
  }, [rosState.playbackScanPointCloud]);

  useEffect(() => {
    if (!rosState.ros) {
      return;
    }

    const cameraTopic = createCurrentCameraPoseTopic(rosState.ros);
    const publish = () => {
      const pose = viewerRef.current?.getCameraPose();
      if (!pose) {
        return;
      }
      publishCurrentCameraPose(cameraTopic, pose);
    };

    publish();
    const intervalId = window.setInterval(publish, 250);
    return () => {
      window.clearInterval(intervalId);
    };
  }, [rosState.ros]);

  useEffect(() => {
    let disposed = false;

    const load = async () => {
      try {
        const response = await fetch(sceneUrl, { cache: "no-store" });
        if (!response.ok) {
          throw new Error(`Failed to load scene manifest: ${response.status}`);
        }
        const nextManifest = (await response.json()) as SceneManifest;
        if (disposed) {
          return;
        }
        setManifest(nextManifest);
        setSceneError(null);
      } catch (error) {
        if (!disposed) {
          setSceneError(error instanceof Error ? error.message : "Failed to load scene manifest.");
        }
      }
    };

    void load();
    const intervalId = window.setInterval(load, 5000);
    return () => {
      disposed = true;
      window.clearInterval(intervalId);
    };
  }, [sceneUrl]);

  useEffect(() => {
    let disposed = false;

    const load = async () => {
      if (!effectiveManifest) {
        return;
      }

      try {
        await viewerRef.current?.loadManifest(effectiveManifest);
        if (!disposed) {
          setSceneError(null);
        }
      } catch (error) {
        if (!disposed) {
          setSceneError(error instanceof Error ? error.message : "Failed to load scene assets.");
        }
      }
    };

    void load();
    return () => {
      disposed = true;
    };
  }, [effectiveManifest]);

  const applyConsoleMode = (nextMode: ConsoleMode) => {
    setConsoleMode(nextMode);
    setLayers((current) => ({
      ...current,
      ...(nextMode === "playback"
        ? layerPresetForPlaybackCloudMode(playbackCloudMode)
        : layerPresetForMode(nextMode))
    }));
  };

  const applyPlaybackCloudMode = (nextMode: PlaybackCloudMode) => {
    setPlaybackCloudMode(nextMode);
    setLayers((current) => ({
      ...current,
      ...layerPresetForPlaybackCloudMode(nextMode)
    }));
  };

  useEffect(() => {
    if (consoleMode !== "playback") {
      return;
    }

    const derived: PlaybackCloudMode = layers.liveCloud
      ? layers.occupancyCloud
        ? "both"
        : "playback"
      : "occupancy";
    if (derived !== playbackCloudMode) {
      setPlaybackCloudMode(derived);
    }
  }, [consoleMode, layers.liveCloud, layers.occupancyCloud, playbackCloudMode]);

  useEffect(() => {
    if (!navPreviewActive) {
      return;
    }

    if (plannedPath.length < 2) {
      setNavPreviewActive(false);
      setNavPreviewPose(null);
      setNavPreviewTrajectory([]);
      setNavPreviewMessage("Preview needs a valid planned path.");
      return;
    }

    const cumulative = buildPathCumulativeLengths(plannedPath);
    const totalLength = cumulative[cumulative.length - 1] ?? 0;
    if (totalLength <= 0.001) {
      setNavPreviewActive(false);
      setNavPreviewPose(null);
      setNavPreviewTrajectory([]);
      setNavPreviewMessage("Preview path is too short.");
      return;
    }

    const frameId = rosState.robotPose?.frameId ?? manifest?.frameId ?? "map";
    const baseZ = rosState.robotPose?.position.z ?? manifest?.meta?.mapOrigin?.z ?? 0.12;
    let travelled = 0;
    let lastTick = performance.now();
    let previewTrace: Pose3D[] = [samplePreviewPose(plannedPath, cumulative, 0, frameId, baseZ)];

    setNavPreviewPose(previewTrace[0]);
    setNavPreviewTrajectory(previewTrace);
    setNavPreviewMessage("Running kinematic nav preview.");

    const intervalId = window.setInterval(() => {
      const now = performance.now();
      const deltaSec = Math.max(0.016, (now - lastTick) / 1000);
      lastTick = now;
      travelled = Math.min(totalLength, travelled + navPreviewSpeed * deltaSec);

      const pose = samplePreviewPose(plannedPath, cumulative, travelled, frameId, baseZ);
      setNavPreviewPose(pose);
      const lastPose = previewTrace[previewTrace.length - 1];
      if (!lastPose || planarPoseDistance(lastPose, pose) >= 0.18 || travelled >= totalLength - 1e-3) {
        previewTrace = [...previewTrace, pose];
        setNavPreviewTrajectory(previewTrace);
      }

      if (travelled >= totalLength - 1e-3) {
        window.clearInterval(intervalId);
        setNavPreviewActive(false);
        setNavPreviewMessage("Preview completed.");
      }
    }, 33);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [
    navPreviewActive,
    navPreviewSpeed,
    plannedPath,
    manifest?.frameId,
    manifest?.meta?.mapOrigin?.z,
    rosState.robotPose?.frameId,
    rosState.robotPose?.position.z
  ]);

  const compareReferenceFrame = rosState.rgbFrame ?? rosState.depthFrame;
  const compareReferenceLabel = rosState.rgbFrame ? "Rendered RGB" : rosState.depthFrame ? "Rendered Depth" : "Reference Render";
  const workspaceModes: ConsoleMode[] = showAdvanced
    ? ["playback", "live", "gs", "compare"]
    : ["playback", "live", "gs"];
  const navTools: Array<readonly [MapTool, string]> = showAdvanced
    ? [
        ["goal", "Goal"],
        ["initialPose", "Initial Pose"],
        ["measure", "Measure"],
        ["obstacle", "Obstacle"],
        ["erase", "Erase"]
      ]
    : [
        ["goal", "Goal"],
        ["initialPose", "Initial Pose"],
        ["obstacle", "Obstacle"],
        ["erase", "Erase"]
      ];

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="topbar-main">
          <div>
            <div className="eyebrow">GS-SDF Console</div>
            <h1>Live Mapping + GS Layer</h1>
          </div>
          <div className="header-tools">
              <div className="mode-cluster">
                <span className="mode-label">Workspace</span>
                <div className="segmented-control">
                {workspaceModes.map((modeOption) => (
                  <button
                    key={modeOption}
                    type="button"
                    className={`segmented-option ${consoleMode === modeOption ? "active" : ""}`}
                    onClick={() => applyConsoleMode(modeOption)}
                  >
                    {titleCase(modeOption)}
                  </button>
                ))}
              </div>
            </div>
            <div className="summary-strip">
              <SummaryCard label="Workspace" value={workspaceSemantic.title} detail={workspaceSemantic.detail} />
              <SummaryCard label="Data Source" value={dataSemantic.title} detail={dataSemantic.detail} />
              <SummaryCard label="Rendering" value={renderSemantic.title} detail={renderSemantic.detail} />
            </div>
          </div>
        </div>

        <div className="topbar-side">
          <div className={`status-chip ${rosState.connectionState}`}>ROS {rosState.connectionState}</div>
          <div className="status-chip neutral">Scene {manifest?.training?.status ?? "loading"}</div>
          <button
            type="button"
            className="link-button"
            onClick={() => setShowAdvanced((current) => !current)}
          >
            {showAdvanced ? "Hide Connection" : "Connection Settings"}
          </button>
          {showAdvanced ? (
            <>
              <label className="endpoint">
                <span>rosbridge</span>
                <input value={wsUrl} onChange={(event) => setWsUrl(event.target.value)} />
              </label>
              {consoleMode === "playback" ? (
                <label className="endpoint">
                  <span>playback-control</span>
                  <input
                    value={playbackApiUrl}
                    onChange={(event) => setPlaybackApiUrl(event.target.value)}
                  />
                </label>
              ) : null}
            </>
          ) : null}
        </div>
      </header>

      <main className="workspace">
        <div className="main-column">
        <section className={`viewer-panel ${consoleMode === "compare" ? "compare-layout" : ""}`}>
          <div className="viewer-stage">
            <canvas ref={canvasRef} className="main-canvas" />
            <div className="viewer-overlay top-left">
              <div className="overlay-card">
                <div className="overlay-title">Main 3D</div>
                <div className="overlay-value">{modeLabel}</div>
                <div className="overlay-hint">
                  {consoleMode === "playback"
                    ? "Bag replay mode. FAST-LIVO2 sidecar publishes a world-frame global map and current scan. Left drag rotates, middle drag pans, wheel zooms."
                    : viewerMode === "live"
                    ? occupancyCloudAssetState === "fallback"
                      ? "Fallback occupancy cloud + geometry overlays. Left drag rotates, middle drag pans, wheel zooms."
                      : "RViz-style working mode. Left drag rotates, middle drag pans, wheel zooms."
                    : gsControlMode === "fps"
                      ? "Photoreal review mode. Click viewport to lock mouse, then use WASD on the ground plane and Q/E for height."
                      : "Photoreal review mode. Left drag rotates, middle drag pans, wheel zooms."}
                </div>
              </div>
            </div>
            <div className="viewer-overlay top-right">
              <div className="overlay-chip-row">
                <span className="overlay-chip">{workspaceSemantic.badge}</span>
                <span className="overlay-chip">{mainViewSource}</span>
              </div>
            </div>
          </div>

          {consoleMode === "compare" ? (
            <div className="compare-rail">
              <section className="compare-card">
                <div className="compare-header">
                  <h2>Reference Render</h2>
                  <span>{compareReferenceLabel}</span>
                </div>
                <CameraFeed
                  title={compareReferenceLabel}
                  frame={compareReferenceFrame}
                  emptyMessage="Waiting for rendered reference frame"
                />
                <p className="hint">
                  Compare the current 3D view against the re-rendered output from `/rviz/current_camera_pose`.
                </p>
              </section>

              <section className="compare-card">
                <div className="compare-header">
                  <h2>Depth Cross-check</h2>
                  <span>{rosState.depthFrame ? `${rosState.depthFrame.width}x${rosState.depthFrame.height}` : "waiting"}</span>
                </div>
                <CameraFeed
                  title="Depth"
                  frame={rosState.depthFrame}
                  emptyMessage="Waiting for depth render"
                />
              </section>

              <section className="compare-card">
                <div className="compare-header">
                  <h2>Alignment Notes</h2>
                </div>
                <div className="kv-list compact">
                  <div>
                    <span>3D mode</span>
                    <strong>{viewerMode === "gs" ? "Gaussian review" : "Geometry review"}</strong>
                  </div>
                  <div>
                    <span>Reference feed</span>
                    <strong>{compareReferenceFrame ? compareReferenceLabel : "waiting"}</strong>
                  </div>
                  <div>
                    <span>Best use</span>
                    <strong>Check floaters, holes, and pose alignment</strong>
                  </div>
                </div>
              </section>
            </div>
          ) : null}
        </section>
        {consoleMode === "playback" ? (
          <section className="timeline-dock">
            <div className="timeline-primary">
              <div className="timeline-status">
                <div className="overlay-title">Playback</div>
                <div className="timeline-title-row">
                  <div className="timeline-title">
                    {playbackStatus?.paused ? "Paused replay" : "Running replay"}
                  </div>
                  <div className="timeline-readout">
                    <strong>{formatSeconds(effectivePlaybackOffset)}</strong>
                    <span>/ {formatSeconds(playbackStatus?.durationSec ?? 0)}</span>
                  </div>
                </div>
              </div>

              <div className="timeline-footer">
                <button
                  type="button"
                  className="primary-button"
                  disabled={!playbackStatus || playbackBusy}
                  onClick={() => {
                    if (!playbackStatus) {
                      return;
                    }
                    void sendPlaybackControl({
                      rate: playbackStatus.rate,
                      paused: !playbackStatus.paused,
                      offsetSec: playbackStatus.currentOffsetSec
                    });
                  }}
                >
                  {playbackStatus?.paused ? "Play" : "Pause"}
                </button>
                <button
                  type="button"
                  className="ghost-button"
                  disabled={!playbackStatus || playbackBusy}
                  onClick={() => {
                    if (!playbackStatus) {
                      return;
                    }
                    void sendPlaybackControl({
                      rate: playbackStatus.rate,
                      paused: playbackStatus.paused,
                      offsetSec: 0
                    });
                  }}
                >
                  Restart
                </button>
              </div>
            </div>

            <input
              className="range-input timeline-slider"
              type="range"
              min={0}
              max={Math.max(playbackStatus?.durationSec ?? 1, 1)}
              step={0.5}
              value={effectivePlaybackOffset}
              disabled={playbackTimelineDisabled}
              onChange={(event) => setPlaybackDraftOffset(Number(event.target.value))}
              onMouseUp={commitPlaybackDraft}
              onTouchEnd={commitPlaybackDraft}
            />

            <div className="timeline-control-grid">
              <div className="segmented-control wrap">
                {playbackRates.map((rate) => (
                  <button
                    key={rate}
                    type="button"
                    className={`segmented-option ${playbackStatus?.rate === rate ? "active" : ""}`}
                    disabled={!playbackStatus || playbackBusy}
                    onClick={() => {
                      if (!playbackStatus) {
                        return;
                      }
                      void sendPlaybackControl({
                        rate,
                        paused: playbackStatus.paused,
                        offsetSec: playbackStatus.currentOffsetSec
                      });
                    }}
                  >
                    {rate}x
                  </button>
                ))}
              </div>

              <div className="timeline-chip-row">
                <span className={`overlay-chip ${playbackStatus?.playbackAlive ? "" : "muted"}`}>
                  {playbackStatus?.playbackAlive ? "rosbag alive" : "rosbag offline"}
                </span>
                <span className="overlay-chip">
                  {playbackStatus ? `${playbackStatus.rate}x` : "waiting"}
                </span>
                <span className="overlay-chip">
                  {playbackStatus?.loop ? "loop" : "single pass"}
                </span>
                <span className="overlay-chip muted">
                  {consoleMode === "playback"
                    ? rosState.playbackAccumulatedPointCloud
                      ? `${rosState.playbackAccumulatedPointCloud.renderedPointCount.toLocaleString()} map pts`
                      : "map waiting"
                    : rosState.livePointCloud
                      ? `${rosState.livePointCloud.renderedPointCount.toLocaleString()} pts`
                      : "cloud waiting"}
                </span>
              </div>
            </div>

            <div className="timeline-controls compact">
              <div className="control-group compact-grow compact-zero">
                <span className="control-label">Cloud Mode</span>
                <div className="segmented-control wrap">
                  {(["playback", "occupancy", "both"] as const).map((mode) => (
                    <button
                      key={mode}
                      type="button"
                      className={`segmented-option ${playbackCloudMode === mode ? "active" : ""}`}
                      onClick={() => applyPlaybackCloudMode(mode)}
                    >
                      {mode === "playback" ? "Scan" : mode === "occupancy" ? "Map" : "Both"}
                    </button>
                  ))}
                </div>
              </div>

              <div className="control-group compact-grow compact-zero">
                <span className="control-label">Playback Cloud</span>
                <div className="segmented-control wrap">
                  {(["fast", "balanced", "quality"] as const).map((profile) => (
                    <button
                      key={profile}
                      type="button"
                      className={`segmented-option ${playbackCloudProfile === profile ? "active" : ""}`}
                      onClick={() => setPlaybackCloudProfile(profile)}
                    >
                      {titleCase(profile)}
                    </button>
                  ))}
                </div>
              </div>
            </div>

            {playbackError ? <p className="error-line">{playbackError}</p> : null}
          </section>
        ) : null}
        </div>

        <aside className="sidebar">
          {consoleMode === "playback" ? (
            <InspectorSection
              title="Playback Video"
              note={rosState.streamProfile === "playback" ? "native playback stream" : "waiting"}
              defaultOpen
            >
              <div className="feed-grid">
                <PlaybackVideoFeed
                  title="Playback RGB"
                  streamUrl={playbackVideoStreamUrl}
                  fallbackFrame={rosState.rgbFrame}
                />
              </div>
            </InspectorSection>
          ) : null}

          <InspectorSection
            title={consoleMode === "playback" ? "Playback Map" : "Navigation Workspace"}
            note={consoleMode === "playback" ? "2D build-up view" : "2D goal / path"}
            defaultOpen
            actions={
              <label className="inline-check">
                <input
                  type="checkbox"
                  checked={followRobot}
                  onChange={(event) => setFollowRobot(event.target.checked)}
                />
                <span>Follow robot</span>
              </label>
            }
          >
            <div className="segmented-control wrap">
              {navTools.map(([toolKey, label]) => (
                <button
                  key={toolKey}
                  type="button"
                  className={`segmented-option ${mapTool === toolKey ? "active" : ""}`}
                  onClick={() => setMapTool(toolKey)}
                >
                  {label}
                </button>
              ))}
            </div>
            <div className="action-row">
              <button
                type="button"
                className="ghost-button"
                onClick={() => {
                  setGoals([]);
                  setPlannedPath([]);
                  setPlannerStatus("idle");
                  setPlannerMessage(null);
                  setMapStatusMessage(null);
                }}
              >
                Clear Goal
              </button>
              <button
                type="button"
                className="ghost-button"
                onClick={() => {
                  setObstacles([]);
                  setErasures([]);
                  setMapStatusMessage("Cleared map edits.");
                }}
              >
                Clear Edits
              </button>
              <button
                type="button"
                className="ghost-button"
                onClick={() => {
                  setMapExportRequestVersion((current) => current + 1);
                  setMapStatusMessage("Exporting edited occupancy map...");
                }}
              >
                Save PGM
              </button>
              {showAdvanced ? (
                <button type="button" className="ghost-button" onClick={() => setMeasurements([])}>
                  Clear Measure
                </button>
              ) : null}
            </div>
            <p className="hint">{toolHint(mapTool)}</p>
            <div className="control-group">
              <span className="control-label">Nav Preview</span>
              <div className="segmented-control wrap">
                {navPreviewSpeeds.map((speed) => (
                  <button
                    key={speed}
                    type="button"
                    className={`segmented-option ${navPreviewSpeed === speed ? "active" : ""}`}
                    onClick={() => setNavPreviewSpeed(speed)}
                  >
                    {speed.toFixed(1)} m/s
                  </button>
                ))}
              </div>
              <div className="action-row compact">
                <button
                  type="button"
                  className="primary-button"
                  disabled={plannedPath.length < 2 || navPreviewActive}
                  onClick={() => {
                    setNavPreviewPose(null);
                    setNavPreviewTrajectory([]);
                    setNavPreviewActive(true);
                  }}
                >
                  {navPreviewActive ? "Preview Running" : "Run Preview"}
                </button>
                <button
                  type="button"
                  className="ghost-button"
                  disabled={!navPreviewActive && navPreviewTrajectory.length === 0}
                  onClick={() => {
                    setNavPreviewActive(false);
                    setNavPreviewPose(null);
                    setNavPreviewTrajectory([]);
                    setNavPreviewMessage("Preview stopped.");
                  }}
                >
                  Stop Preview
                </button>
              </div>
            </div>
            <div className="kv-list compact nav-summary">
              <div>
                <span>Planner</span>
                <strong>{plannerStatus === "ready" ? "Local A*" : plannerStatus === "blocked" ? "Blocked" : "Idle"}</strong>
              </div>
              <div>
                <span>Path preview</span>
                <strong>{plannedPath.length > 1 ? `${pathLengthMeters(plannedPath).toFixed(1)} m` : "none"}</strong>
              </div>
              <div>
                <span>Nav preview</span>
                <strong>{navPreviewActive ? "running" : navPreviewTrajectory.length > 1 ? "ready" : "idle"}</strong>
              </div>
            </div>
            {plannerMessage ? <p className="hint">{plannerMessage}</p> : null}
            {mapStatusMessage ? <p className="hint">{mapStatusMessage}</p> : null}
            {navPreviewMessage ? <p className="hint">{navPreviewMessage}</p> : null}
            <div className="mini-map">
              <TopDownMap
                manifest={manifest}
                ros={rosState.ros}
                streamProfile={rosState.streamProfile}
                robotPose={displayRobotPose}
                trajectory={displayTrajectory}
                mapPointCloud={activeMapPointCloud}
                goals={goals}
                plannedPath={plannedPath}
                initialPose={initialPose}
                measurements={measurements}
                obstacles={obstacles}
                erasures={erasures}
                selectedTool={mapTool}
                followRobot={followRobot}
                exportRequestVersion={mapExportRequestVersion}
                onGoal={(goal) => setGoals((current) => [...current.slice(-19), goal])}
                onPlannedPath={setPlannedPath}
                onPlanningState={(status, reason) => {
                  setPlannerStatus(status);
                  setPlannerMessage(reason ?? null);
                }}
                onInitialPose={(pose) => setInitialPose(pose)}
                onMeasurement={(measurement) => setMeasurements((current) => [...current.slice(-11), measurement])}
                onObstacle={(obstacle) => setObstacles((current) => [...current.slice(-11), obstacle])}
                onErase={(eraseRect) => setErasures((current) => [...current.slice(-11), eraseRect])}
                onExported={(message, ok) => setMapStatusMessage(ok ? message : `Export failed: ${message}`)}
              />
            </div>
            <p className="hint">
              {consoleMode === "playback"
                ? "Use this as a replay overview. Goal, obstacle, and erase tools edit the 2D occupancy map directly before exporting PGM."
                : "Wheel to zoom. Shift + drag to pan. Goal publishes to ROS and local planning uses the edited occupancy map."}
            </p>
            <p className="hint">Nav Preview is a kinematic UI replay along the planned path, not a full Gazebo/Nav2 physics simulation.</p>
          </InspectorSection>

          {consoleMode !== "compare" && consoleMode !== "playback" && showAdvanced ? (
            <InspectorSection title="Camera Feeds" note="rendered from web camera pose" defaultOpen={false}>
              <div className="feed-grid">
                <CameraFeed title="RGB" frame={rosState.rgbFrame} />
                <CameraFeed title="Depth" frame={rosState.depthFrame} />
              </div>
              <p className="hint">
                These feeds render from the current Web camera pose through `/rviz/current_camera_pose`.
              </p>
            </InspectorSection>
          ) : null}

          <InspectorSection title="Scene Controls" note={mainViewSource} defaultOpen>
            <div className="toggle-list">
              {(
                [
                  ["gaussian", "Gaussian"],
                  ["sdfMesh", "SDF Mesh"],
                  ["occupancyCloud", occupancyCloudLabel],
                  ["liveCloud", liveCloudLabel],
                  ["trajectory", "Trajectory"],
                  ["robot", "Robot"]
                ] as const
              ).map(([key, label]) => (
                <label key={key} className="toggle-item">
                  <input
                    type="checkbox"
                    checked={layers[key]}
                    onChange={(event) =>
                      setLayers((current) => ({
                        ...current,
                        [key]: event.target.checked
                      }))
                    }
                  />
                  <span>{label}</span>
                  <span className="asset-state">
                    {layerAssetState(key, manifest, hasLiveCloud, consoleMode)}
                  </span>
                </label>
              ))}
            </div>
            <div className="control-stack">
              <div className="control-group">
                <span className="control-label">Gaussian quality</span>

                {gaussianVariants.length > 1 ? (
                  <div className="segmented-control wrap">
                    {gaussianVariants.map(([key, source]) => (
                      <button
                        key={key}
                        type="button"
                        className={`segmented-option ${activeGaussianVariant === key ? "active" : ""}`}
                        onClick={() => setGaussianVariant(key)}
                      >
                        {source.label ?? titleCase(key)}
                      </button>
                    ))}
                  </div>
                ) : (
                  <strong className="inline-value">{titleCase(activeGaussianVariant)}</strong>
                )}
              </div>

              <div className="control-group">
                <span className="control-label">GS controls</span>
                <div className="segmented-control">
                  <button
                    type="button"
                    className={`segmented-option ${gsControlMode === "orbit" ? "active" : ""}`}
                    onClick={() => setGsControlMode("orbit")}
                  >
                    Orbit
                  </button>
                  <button
                    type="button"
                    className={`segmented-option ${gsControlMode === "fps" ? "active" : ""}`}
                    onClick={() => setGsControlMode("fps")}
                  >
                    FPS
                  </button>
                </div>
              </div>

              {consoleMode === "playback" ? (
                <div className="control-group">
                  <span className="control-label">Playback decay</span>
                  <div className="segmented-control wrap">
                    {playbackDecayOptions.map((seconds) => (
                      <button
                        key={seconds}
                        type="button"
                        className={`segmented-option ${playbackDecaySec === seconds ? "active" : ""}`}
                        onClick={() => setPlaybackDecaySec(seconds)}
                      >
                        {seconds === 0 ? "Off" : `${seconds}s`}
                      </button>
                    ))}
                  </div>
                </div>
              ) : null}
            </div>
          </InspectorSection>

          {showAdvanced ? (
            <InspectorSection
              title="System State"
              note={manifest?.frameId ?? "world"}
              defaultOpen={false}
            >
              {sceneError ? <p className="error-line">{sceneError}</p> : null}
              {rosState.errorMessage ? <p className="error-line">{rosState.errorMessage}</p> : null}
              <div className="kv-list">
                <div>
                  <span>Workspace mode</span>
                  <strong>{workspaceSemantic.title}</strong>
                </div>
                <div>
                  <span>Data source</span>
                  <strong>{dataSemantic.title}</strong>
                </div>
                <div>
                  <span>Main 3D source</span>
                  <strong>{mainViewSource}</strong>
                </div>
                <div>
                  <span>Trajectory points</span>
                  <strong>{rosState.trajectory.length}</strong>
                </div>
                <div>
                  <span>Live cloud</span>
                  <strong>
                    {consoleMode === "playback" && rosState.playbackAccumulatedPointCloud
                      ? `${rosState.playbackAccumulatedPointCloud.renderedPointCount.toLocaleString()} map / ${rosState.playbackScanPointCloud?.renderedPointCount.toLocaleString() ?? 0} scan`
                      : rosState.livePointCloud
                        ? `${rosState.livePointCloud.renderedPointCount.toLocaleString()} / ${rosState.livePointCloud.sourcePointCount.toLocaleString()}`
                      : "waiting"}
                  </strong>
                </div>
                <div>
                  <span>Goals</span>
                  <strong>{goals.length}</strong>
                </div>
                <div>
                  <span>Initial pose</span>
                  <strong>{initialPose ? "set" : "unset"}</strong>
                </div>
                <div>
                  <span>Measurements</span>
                  <strong>{measurements.length}</strong>
                </div>
                <div>
                  <span>Obstacles</span>
                  <strong>{obstacles.length}</strong>
                </div>
                <div>
                  <span>Erasures</span>
                  <strong>{erasures.length}</strong>
                </div>
                <div>
                  <span>Gaussian asset</span>
                  <strong>{manifest?.assets?.gaussian ?? "loading"}</strong>
                </div>
                <div>
                  <span>Gaussian SH</span>
                  <strong>{effectiveManifest?.gaussian?.shDegree ?? manifest?.gaussian?.shDegree ?? "n/a"}</strong>
                </div>
                <div>
                  <span>Occupancy/static cloud asset</span>
                  <strong>{manifest?.assets?.rawPointCloud ?? "loading"}</strong>
                </div>
                <div>
                  <span>Live playback cloud</span>
                  <strong>{hasLiveCloud ? "streaming" : "waiting"}</strong>
                </div>
              </div>
            </InspectorSection>
          ) : null}
        </aside>
      </main>
    </div>
  );
}

function getGaussianVariants(manifest: SceneManifest | null): Array<[string, GaussianSource]> {
  if (!manifest?.gaussianVariants) {
    if (!manifest?.gaussian) {
      return [];
    }
    const variant = manifest.gaussian.variant ?? "balanced";
    return [[variant, manifest.gaussian]];
  }

  return Object.entries(manifest.gaussianVariants).sort((left, right) => {
    return variantPriority(left[0]) - variantPriority(right[0]);
  });
}

function variantPriority(variant: string): number {
  switch (variant) {
    case "fast":
      return 0;
    case "balanced":
      return 1;
    case "quality":
      return 2;
    default:
      return 10;
  }
}

function resolveGaussianVariant(manifest: SceneManifest | null, variant: string | null): SceneManifest | null {
  if (!manifest || !manifest.gaussianVariants || !variant || !manifest.gaussianVariants[variant]) {
    return manifest;
  }

  return {
    ...manifest,
    gaussian: manifest.gaussianVariants[variant]
  };
}

function layerPresetForMode(mode: ConsoleMode): LayerVisibility {
  switch (mode) {
    case "playback":
      return {
        gaussian: false,
        sdfMesh: false,
        occupancyCloud: true,
        liveCloud: true,
        trajectory: true,
        robot: true
      };
    case "live":
      return {
        gaussian: false,
        sdfMesh: true,
        occupancyCloud: true,
        liveCloud: true,
        trajectory: true,
        robot: true
      };
    case "gs":
    case "compare":
      return {
        gaussian: true,
        sdfMesh: true,
        occupancyCloud: false,
        liveCloud: false,
        trajectory: true,
        robot: true
      };
    default:
      return {
        gaussian: false,
        sdfMesh: true,
        occupancyCloud: true,
        liveCloud: true,
        trajectory: true,
        robot: true
      };
  }
}

function layerPresetForPlaybackCloudMode(mode: PlaybackCloudMode): Pick<LayerVisibility, "occupancyCloud" | "liveCloud"> {
  switch (mode) {
    case "occupancy":
      return {
        occupancyCloud: true,
        liveCloud: false
      };
    case "both":
      return {
        occupancyCloud: true,
        liveCloud: true
      };
    case "playback":
    default:
      return {
        occupancyCloud: false,
        liveCloud: true
      };
  }
}

function describeWorkspaceSemantic(
  consoleMode: ConsoleMode,
  viewerMode: ViewerMode,
  rawAssetState: string,
  streamProfile: "idle" | "neural" | "playback"
) {
  if (consoleMode === "playback") {
    return {
      title: "Playback mapping",
      detail:
        streamProfile === "playback"
          ? "FAST-LIVO2 bag replay with progressive cloud growth"
          : "Waiting for raw bag playback topics",
      badge: "Playback"
    };
  }

  if (consoleMode === "compare") {
    return {
      title: "Compare review",
      detail: "3D scene next to rendered references",
      badge: "Compare"
    };
  }

  if (viewerMode === "gs") {
    return {
      title: "GS review",
      detail: "Photoreal gaussian inspection and walkthrough",
      badge: "GS"
    };
  }

  return {
    title: rawAssetState === "fallback" ? "Live geometry" : "Live mapping",
    detail: rawAssetState === "fallback" ? "Fallback geometry with nav overlays" : "RViz-style geometry and tracking",
    badge: rawAssetState === "fallback" ? "Fallback" : "Live"
  };
}

function describeDataSemantic(
  connectionState: string,
  streamProfile: "idle" | "neural" | "playback",
  livePointCloud: { renderedPointCount: number } | null,
  rawAssetState: string
) {
  if (connectionState === "connected" && streamProfile === "playback" && livePointCloud) {
    return {
      title: "FAST-LIVO2 bag",
      detail: `${livePointCloud.renderedPointCount.toLocaleString()} replay points in memory`
    };
  }

  if (connectionState === "connected" && streamProfile === "playback") {
    return {
      title: "Playback connected",
      detail: "Waiting for /fastlivo/global_map, /fastlivo/current_scan_world, and /origin_img"
    };
  }

  if (connectionState === "connected" && livePointCloud) {
    return {
      title: "ROS live",
      detail: `${livePointCloud.renderedPointCount.toLocaleString()} live points in memory`
    };
  }

  if (connectionState === "connected") {
    return {
      title: "ROS connected",
      detail: "Topics connected; waiting for live cloud or using saved view topics"
    };
  }

  if (rawAssetState === "fallback") {
    return {
      title: "Saved scene",
      detail: "Static occupancy fallback instead of live point cloud"
    };
  }

  return {
    title: "Scene only",
    detail: "Static assets loaded without active live topic flow"
  };
}

function describeRenderSemantic(
  consoleMode: ConsoleMode,
  viewerMode: ViewerMode,
  gaussianVariant: string,
  gsControlMode: GsControlMode
) {
  if (consoleMode === "playback") {
    return {
      title: "Progressive playback",
      detail: "Raw point cloud + RGB from bag replay"
    };
  }

  if (viewerMode === "live") {
    return {
      title: "Geometry-first",
      detail: "Orbit / pan for mapping and nav overlays"
    };
  }

  return {
    title: `${titleCase(gaussianVariant)} / ${gsControlMode === "fps" ? "FPS" : "Orbit"}`,
    detail: "Gaussian-first rendering with deferred loading"
  };
}

function titleCase(value: string): string {
  return value
    .replace(/[_-]+/g, " ")
    .replace(/\b\w/g, (character) => character.toUpperCase());
}

function layerAssetState(
  key: keyof LayerVisibility,
  manifest: SceneManifest | null,
  hasLiveCloud: boolean,
  consoleMode: ConsoleMode
): string {
  switch (key) {
    case "sdfMesh":
      return manifest?.assets?.mesh ?? "n/a";
    case "occupancyCloud":
      return consoleMode === "playback" ? (hasLiveCloud ? "streaming" : "waiting") : manifest?.assets?.rawPointCloud ?? "n/a";
    case "liveCloud":
      return hasLiveCloud ? "streaming" : "waiting";
    default:
      return manifest?.assets?.[key] ?? "n/a";
  }
}

function toolHint(tool: MapTool): string {
  switch (tool) {
    case "goal":
      return "Click to publish /move_base_simple/goal.";
    case "initialPose":
      return "Click to publish /initialpose for localization reset.";
    case "measure":
      return "Drag to measure planar distance on the occupancy grid.";
    case "obstacle":
      return "Drag to paint occupied cells into the editable map.";
    case "erase":
      return "Drag to clear occupied cells and remove noise from the editable map.";
    default:
      return "";
  }
}

function pathLengthMeters(path: PlanPoint2D[]): number {
  let total = 0;
  for (let index = 1; index < path.length; index += 1) {
    total += Math.hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y);
  }
  return total;
}

function buildPathCumulativeLengths(path: PlanPoint2D[]): number[] {
  const cumulative = new Array<number>(path.length).fill(0);
  for (let index = 1; index < path.length; index += 1) {
    cumulative[index] =
      cumulative[index - 1] +
      Math.hypot(path[index].x - path[index - 1].x, path[index].y - path[index - 1].y);
  }
  return cumulative;
}

function samplePreviewPose(
  path: PlanPoint2D[],
  cumulative: number[],
  distance: number,
  frameId: string,
  z: number
): Pose3D {
  if (path.length === 0) {
    return {
      position: { x: 0, y: 0, z },
      orientation: yawToQuaternion(0),
      frameId
    };
  }

  if (path.length === 1 || distance <= 0) {
    const next = path[Math.min(1, path.length - 1)];
    const yaw = Math.atan2(next.y - path[0].y, next.x - path[0].x);
    return {
      position: { x: path[0].x, y: path[0].y, z },
      orientation: yawToQuaternion(Number.isFinite(yaw) ? yaw : 0),
      frameId
    };
  }

  const totalLength = cumulative[cumulative.length - 1] ?? 0;
  const clampedDistance = Math.min(Math.max(distance, 0), totalLength);

  for (let index = 1; index < path.length; index += 1) {
    if (clampedDistance > cumulative[index]) {
      continue;
    }

    const segmentStart = path[index - 1];
    const segmentEnd = path[index];
    const segmentStartDistance = cumulative[index - 1];
    const segmentLength = Math.max(1e-6, cumulative[index] - segmentStartDistance);
    const segmentT = (clampedDistance - segmentStartDistance) / segmentLength;
    const x = segmentStart.x + (segmentEnd.x - segmentStart.x) * segmentT;
    const y = segmentStart.y + (segmentEnd.y - segmentStart.y) * segmentT;
    const yaw = Math.atan2(segmentEnd.y - segmentStart.y, segmentEnd.x - segmentStart.x);
    return {
      position: { x, y, z },
      orientation: yawToQuaternion(Number.isFinite(yaw) ? yaw : 0),
      frameId
    };
  }

  const last = path[path.length - 1];
  const prev = path[path.length - 2] ?? last;
  const yaw = Math.atan2(last.y - prev.y, last.x - prev.x);
  return {
    position: { x: last.x, y: last.y, z },
    orientation: yawToQuaternion(Number.isFinite(yaw) ? yaw : 0),
    frameId
  };
}

function yawToQuaternion(yaw: number): Pose3D["orientation"] {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw * 0.5),
    w: Math.cos(yaw * 0.5)
  };
}

function planarPoseDistance(left: Pose3D, right: Pose3D): number {
  return Math.hypot(
    right.position.x - left.position.x,
    right.position.y - left.position.y
  );
}

function formatPoseStamp(pose: { stampMs?: number } | null): string {
  if (!pose?.stampMs) {
    return "waiting";
  }
  return `${(pose.stampMs / 1000).toFixed(2)} s`;
}

function nearestPlaybackRateIndex(rate: number): number {
  let bestIndex = 0;
  let bestDistance = Number.POSITIVE_INFINITY;

  for (let index = 0; index < playbackRates.length; index += 1) {
    const distance = Math.abs(playbackRates[index] - rate);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = index;
    }
  }

  return bestIndex;
}

function formatSeconds(value: number): string {
  const totalSeconds = Math.max(0, Math.floor(value));
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}:${seconds.toString().padStart(2, "0")}`;
}

function buildPlaybackVideoUrl(playbackApiUrl: string, path: string): string | null {
  try {
    const url = new URL(playbackApiUrl);
    url.pathname = path;
    url.search = "";
    return url.toString();
  } catch (error) {
    console.warn("Failed to build playback video URL.", error);
    return null;
  }
}

function SummaryCard(props: { label: string; value: string; detail: string }) {
  return (
    <div className="summary-card">
      <span>{props.label}</span>
      <strong>{props.value}</strong>
      <small>{props.detail}</small>
    </div>
  );
}

function InspectorSection(props: {
  title: string;
  note?: string;
  defaultOpen?: boolean;
  actions?: ReactNode;
  children: ReactNode;
}) {
  return (
    <details className="inspector-section panel" open={props.defaultOpen}>
      <summary className="panel-summary">
        <div className="panel-summary-main">
          <h2>{props.title}</h2>
          {props.note ? <span className="panel-note">{props.note}</span> : null}
        </div>
        <div className="panel-summary-side">
          {props.actions}
          <span className="summary-caret">›</span>
        </div>
      </summary>
      <div className="panel-body">{props.children}</div>
    </details>
  );
}

function CameraFeed(props: { title: string; frame: DecodedRosImage | null; emptyMessage?: string }) {
  const { frame, title, emptyMessage } = props;
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !frame) {
      return;
    }

    if (canvas.width !== frame.width || canvas.height !== frame.height) {
      canvas.width = frame.width;
      canvas.height = frame.height;
    }

    const context = canvas.getContext("2d");
    if (!context) {
      return;
    }

    const imageData = context.createImageData(frame.width, frame.height);
    imageData.data.set(frame.rgba);
    context.putImageData(imageData, 0, 0);
  }, [frame]);

  return (
    <div className="feed-tile">
      <div className="feed-header">
        <span>{title}</span>
        <strong>{frame ? `${frame.width}x${frame.height}` : "waiting"}</strong>
      </div>
      <div className="feed-frame">
        {frame ? (
          <canvas ref={canvasRef} className="feed-canvas" aria-label={title} />
        ) : (
          <div className="feed-empty">{emptyMessage ?? `Waiting for \`${title}\` frame`}</div>
        )}
      </div>
    </div>
  );
}

function PlaybackVideoFeed(props: {
  title: string;
  streamUrl: string | null;
  fallbackFrame: DecodedRosImage | null;
  emptyMessage?: string;
}) {
  const { title, streamUrl, fallbackFrame, emptyMessage } = props;
  const [streamFailed, setStreamFailed] = useState(false);

  useEffect(() => {
    setStreamFailed(false);
  }, [streamUrl]);

  if (!streamUrl || streamFailed) {
    return (
      <CameraFeed
        title={title}
        frame={fallbackFrame}
        emptyMessage={emptyMessage ?? `Waiting for \`${title}\` frame`}
      />
    );
  }

  return (
    <div className="feed-tile">
      <div className="feed-header">
        <span>{title}</span>
        <strong>{fallbackFrame ? `${fallbackFrame.width}x${fallbackFrame.height}` : "stream"}</strong>
      </div>
      <div className="feed-frame">
        <img
          src={streamUrl}
          alt={title}
          onError={() => setStreamFailed(true)}
        />
      </div>
    </div>
  );
}
