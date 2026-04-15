import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import type { GaussianSource, LayerVisibility, Pose3D, SceneManifest } from "./lib/gs/gsSdfSceneAdapter";
import {
  createCurrentCameraPoseTopic,
  publishCurrentCameraPose,
  type GoalPose2D,
  type InitialPose2D
} from "./lib/ros/navGoalPublisher";
import type { DecodedPointCloud } from "./lib/ros/pointCloud2";
import type { DecodedRosImage } from "./lib/ros/rosImage";
import { useNeuralMappingRos, type PlaybackCloudOptions } from "./lib/ros/useNeuralMappingRos";
import {
  LiveGsViewer,
  type GsControlMode,
  type PresentationMode,
  type ViewerMode
} from "./lib/viewer/liveGsViewer";
import type { PlanPoint2D } from "./lib/map/localPlanner";
import TopDownMap, {
  type MapTool,
  type Measurement2D,
  type ObstacleRect2D
} from "./components/TopDownMap";

type ConsoleMode = "playback" | "live" | "gs" | "hifi" | "compare";
type PlaybackCloudMode = "playback" | "occupancy" | "both";
type PlaybackCloudProfile = "fast" | "balanced" | "quality";
type PlaybackCompareMode = "video" | "live" | "webgs";
type PlaybackPresentation = "workspace" | "showcase";
const playbackShowcaseEnabled = true;

const defaultSceneUrl = "/scenes/fast-livo2-compressed-live/manifest.json";
const fallbackPlaybackVideoSyncConfig = {
  fps: 12,
  speed: 8,
  startOffsetSec: 0,
  endOffsetSec: -1
} as const;
const playbackVideoRates = [1, 2, 4] as const;
const playbackDecayOptions = [0, 5, 15, 30] as const;
const navPreviewSpeeds = [0.6, 1.0, 1.6] as const;
const HIFI_DEFAULT_CAMERA_POSE: Pose3D = {
  frameId: "world",
  position: {
    x: 10.886779744651108,
    y: 8.276177854283173,
    z: 17.233192011804636
  },
  orientation: {
    x: 0.14624079594136272,
    y: 0.2948731503353518,
    z: 0.8459568712879663,
    w: 0.41954788372057955
  }
};
const HIFI_DEFAULT_TARGET = { x: 0, y: 0, z: 0 };
const playbackCloudProfiles: Record<
  PlaybackCloudProfile,
  Pick<
    PlaybackCloudOptions,
    "maxInputPoints" | "maxAccumulatedPoints" | "voxelSize" | "publishEveryFrames" | "minVoxelObservations"
  >
> = {
  fast: {
    maxInputPoints: 2200,
    maxAccumulatedPoints: 120_000,
    voxelSize: 0.3,
    publishEveryFrames: 1,
    minVoxelObservations: 3
  },
  balanced: {
    maxInputPoints: 3200,
    maxAccumulatedPoints: 220_000,
    voxelSize: 0.24,
    publishEveryFrames: 1,
    minVoxelObservations: 2
  },
  quality: {
    maxInputPoints: 7600,
    maxAccumulatedPoints: 480_000,
    voxelSize: 0.15,
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
  /** Bag time at the last fully published frame (steps when new frames land). */
  currentOffsetSec: number;
  /** Bag time extrapolated with wall clock × rate while playing (matches real-time UI). */
  estimatedOffsetSec?: number;
  durationSec: number;
  bagStartSec: number;
  bagEndSec: number;
  playbackAlive?: boolean;
  latestImageStampSec?: number | null;
  latestPoseStampSec?: number | null;
  globalMapPoints?: number;
  currentScanPoints?: number;
  frameIndex?: number;
  frameCount?: number;
  keyframeCount?: number;
  cacheDir?: string;
  currentPose?: Pose3D | null;
  video?: PlaybackVideoDescriptor | null;
  mode?: string;
}

interface PlaybackVideoDescriptor {
  url: string;
  fps?: number;
  speed?: number;
  startOffsetSec?: number;
  endOffsetSec?: number;
  label?: string;
}

interface PlaybackVideoSyncMap {
  mode: string;
  cacheDir?: string;
  fps: number;
  speed: number;
  videoFrameCount: number;
  playbackFrameCount: number;
  offsetsSec: number[];
  frameIndices: number[];
  videoDurationSec: number;
  playbackDurationSec: number;
}

interface PlaybackOverlayScanPayload {
  frameId?: string;
  stampMs?: number;
  renderedPointCount?: number;
  sourcePointCount?: number;
  positions?: number[];
  colors?: number[];
}

interface PlaybackOverlayState {
  frameIndex: number;
  trajectory: Pose3D[];
  scan: PlaybackOverlayScanPayload | null;
  map?: PlaybackOverlayScanPayload | null;
}

interface PlaybackPoseTrack {
  mode: string;
  cacheDir?: string;
  frameCount: number;
  poses: Pose3D[];
}

interface PlaybackLocalMapVoxel {
  x: number;
  y: number;
  z: number;
  positionSamples: number;
  color?: [number, number, number];
  colorSamples: number;
}

interface PlaybackLocalMapState {
  voxels: Map<string, PlaybackLocalMapVoxel>;
  order: string[];
  lastFrameIndex: number | null;
}

interface HiFiStatus {
  mode: string;
  ready: boolean;
  imageTopic: string;
  poseTopic: string;
  sourceContainer: string;
  image: string;
  latestFrameStampSec: number | null;
  frameWidth: number;
  frameHeight: number;
  previewFrameWidth?: number;
  previewFrameHeight?: number;
  framesReceived: number;
  poseMessagesSent: number;
  lastPoseWallSec: number | null;
}

interface HiFiWindowStatus {
  mode: string;
  ready: boolean;
  display: string;
  width: number;
  height: number;
  framesCaptured: number;
  latestFrameWallSec: number | null;
  lastError: string | null;
  label: string;
}

export default function App() {
  const searchParams = useMemo(() => new URLSearchParams(window.location.search), []);
  const sceneUrl = useMemo(
    () => searchParams.get("scene") ?? defaultSceneUrl,
    [searchParams]
  );
  const initialConsoleMode = useMemo<ConsoleMode>(() => {
    const requested = searchParams.get("mode");
    if (requested === "playback" || requested === "gs" || requested === "hifi" || requested === "compare") {
      return requested;
    }
    return "live";
  }, [searchParams]);
  const initialGsControlMode = useMemo<GsControlMode>(
    () => (searchParams.get("controls") === "fps" ? "fps" : "orbit"),
    [searchParams]
  );
  const initialGaussianVariant = useMemo(
    () => searchParams.get("gaussian"),
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
  const defaultWsUrl = useMemo(() => {
    const scheme = window.location.protocol === "https:" ? "wss" : "ws";
    return `${scheme}://${window.location.host}/__rosbridge`;
  }, []);
  const defaultPlaybackApiUrl = useMemo(() => "/__playback_control", []);
  const defaultHiFiApiUrl = useMemo(() => "/__hifi_bridge", []);
  const defaultHiFiStreamUrl = useMemo(() => "/__hifi_window", []);

  const [wsUrl, setWsUrl] = useState(defaultWsUrl);
  const [playbackApiUrl, setPlaybackApiUrl] = useState(defaultPlaybackApiUrl);
  const [hifiApiUrl, setHifiApiUrl] = useState(defaultHiFiApiUrl);
  const [hifiStreamUrlBase, setHiFiStreamUrlBase] = useState(defaultHiFiStreamUrl);
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
  const [playbackVideoReady, setPlaybackVideoReady] = useState(false);
  const [playbackVideoDurationSec, setPlaybackVideoDurationSec] = useState(0);
  const [playbackVideoCurrentTimeSec, setPlaybackVideoCurrentTimeSec] = useState(0);
  const [playbackVideoPresentedTimeSec, setPlaybackVideoPresentedTimeSec] = useState<number | null>(null);
  const [playbackVideoSyncFrameIndex, setPlaybackVideoSyncFrameIndex] = useState<number | null>(null);
  const [playbackVideoPaused, setPlaybackVideoPaused] = useState(true);
  const [playbackVideoRate, setPlaybackVideoRate] = useState(1);
  const [playbackVideoDraftTimeSec, setPlaybackVideoDraftTimeSec] = useState<number | null>(null);
  const [playbackVideoSyncMap, setPlaybackVideoSyncMap] = useState<PlaybackVideoSyncMap | null>(null);
  const [playbackPoseTrack, setPlaybackPoseTrack] = useState<PlaybackPoseTrack | null>(null);
  const [playbackCompareMode, setPlaybackCompareMode] = useState<PlaybackCompareMode>("video");
  const [playbackPresentation, setPlaybackPresentation] = useState<PlaybackPresentation>("workspace");
  const [playbackOverlayTrajectory, setPlaybackOverlayTrajectory] = useState<Pose3D[] | null>(null);
  const [playbackOverlayScanPointCloud, setPlaybackOverlayScanPointCloud] = useState<DecodedPointCloud | null>(null);
  const [playbackOverlayFrameIndex, setPlaybackOverlayFrameIndex] = useState<number | null>(null);
  const [playbackOverlayAccumulatedPointCloud, setPlaybackOverlayAccumulatedPointCloud] =
    useState<DecodedPointCloud | null>(null);
  const playbackStreamSourcePoseRef = useRef<Pose3D | null>(null);
  const playbackHiFiPoseBusyRef = useRef(false);
  const hifiManifestCameraRef = useRef<SceneManifest["camera"] | undefined>(undefined);
  const [hifiStatus, setHifiStatus] = useState<HiFiStatus | null>(null);
  const [hifiWindowStatus, setHifiWindowStatus] = useState<HiFiWindowStatus | null>(null);
  const [hifiError, setHifiError] = useState<string | null>(null);
  const [hifiStreamFailed, setHifiStreamFailed] = useState(false);
  const [hifiInteractive, setHifiInteractive] = useState(false);
  const playbackStatusRequestInFlightRef = useRef(false);
  const playbackStatusFailureCountRef = useRef(0);
  const playbackStatusRef = useRef<PlaybackStatus | null>(null);
  const hifiPoseRequestInFlightRef = useRef(false);
  const hifiInitializedRef = useRef(false);
  const hifiLastPublishedPoseRef = useRef<Pose3D | null>(null);
  const hifiLastPublishedAtRef = useRef(0);
  const playbackVideoRef = useRef<HTMLVideoElement | null>(null);
  const [playbackVideoElement, setPlaybackVideoElement] = useState<HTMLVideoElement | null>(null);
  const playbackVideoCarryoverRef = useRef<{
    currentTimeSec: number;
    paused: boolean;
    playbackRate: number;
  } | null>(null);
  const playbackVideoCarryoverPendingRef = useRef(false);
  const playbackVideoInitializedRef = useRef(false);
  const playbackVideoSyncInFlightRef = useRef(false);
  const playbackVideoPendingSyncRef = useRef(false);
  const playbackVideoLastSyncSignatureRef = useRef<string | null>(null);
  const playbackVideoPlayPromiseRef = useRef<Promise<void> | null>(null);
  const playbackVideoScrubbingRef = useRef(false);
  const playbackVideoScrubResumeRef = useRef(false);
  const playbackVideoLastFrameSyncRef = useRef<number | null>(null);
  const playbackVideoLastSyncWallRef = useRef(0);
  const playbackVideoLastSeekWallRef = useRef(0);
  const playbackVideoInitializingRef = useRef(false);
  const playbackOverlayAbortRef = useRef<AbortController | null>(null);
  const playbackOverlayFetchInFlightRef = useRef(false);
  const playbackOverlayDesiredRef = useRef<{ frameIndex: number; tailSec: number } | null>(null);
  const playbackOverlayLoadedKeyRef = useRef<string | null>(null);
  const playbackOverlayLastRequestedFrameRef = useRef<number | null>(null);
  const playbackOverlayLastRequestWallRef = useRef(0);
  const playbackOverlayAccumulationRef = useRef<PlaybackLocalMapState>({
    voxels: new Map(),
    order: [],
    lastFrameIndex: null
  });
  const sceneManifestSignatureRef = useRef<string | null>(null);
  const playbackVideoCurrentTimeStateRef = useRef(0);
  const playbackVideoPresentedTimeStateRef = useRef<number | null>(null);
  const playbackVideoPausedStateRef = useRef(true);
  const playbackVideoRateStateRef = useRef(1);
  const playbackVideoDraftTimeStateRef = useRef<number | null>(null);
  const [mainCanvasNode, setMainCanvasNode] = useState<HTMLCanvasElement | null>(null);
  const setMainCanvasRef = useCallback((node: HTMLCanvasElement | null) => {
    setMainCanvasNode(node);
  }, []);
  const viewerRef = useRef<LiveGsViewer | null>(null);
  const [playbackResolvedMapPointCloud, setPlaybackResolvedMapPointCloud] =
    useState<DecodedPointCloud | null>(null);

  useEffect(() => {
    playbackStatusRef.current = playbackStatus;
  }, [playbackStatus]);

  const commitPlaybackStatus = useCallback((nextStatus: PlaybackStatus) => {
    setPlaybackStatus((current) =>
      arePlaybackStatusesEquivalent(current, nextStatus) ? current : nextStatus
    );
  }, []);

  useEffect(() => {
    playbackVideoCurrentTimeStateRef.current = playbackVideoCurrentTimeSec;
  }, [playbackVideoCurrentTimeSec]);

  useEffect(() => {
    playbackVideoPresentedTimeStateRef.current = playbackVideoPresentedTimeSec;
  }, [playbackVideoPresentedTimeSec]);

  useEffect(() => {
    playbackVideoPausedStateRef.current = playbackVideoPaused;
  }, [playbackVideoPaused]);

  useEffect(() => {
    playbackVideoRateStateRef.current = playbackVideoRate;
  }, [playbackVideoRate]);

  useEffect(() => {
    playbackVideoDraftTimeStateRef.current = playbackVideoDraftTimeSec;
  }, [playbackVideoDraftTimeSec]);

  const refreshPlaybackVideoState = useCallback(() => {
    const video = playbackVideoRef.current;
    if (!video) {
      setPlaybackVideoReady(false);
      setPlaybackVideoDurationSec(0);
      setPlaybackVideoCurrentTimeSec(0);
      setPlaybackVideoPresentedTimeSec(null);
      setPlaybackVideoPaused(true);
      setPlaybackVideoRate(1);
      return;
    }

    const durationSec = Number.isFinite(video.duration) ? Math.max(video.duration, 0) : 0;
    setPlaybackVideoReady(video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && durationSec > 0);
    setPlaybackVideoDurationSec(durationSec);
    setPlaybackVideoCurrentTimeSec(Math.max(0, video.currentTime || 0));
    setPlaybackVideoPaused(video.paused || video.ended);
    setPlaybackVideoRate(video.playbackRate || 1);
  }, []);

  const setPlaybackVideoNode = useCallback(
    (node: HTMLVideoElement | null) => {
      const previous = playbackVideoRef.current;
      if (previous && previous !== node) {
        playbackVideoCarryoverRef.current = {
          currentTimeSec: Math.max(
            0,
            playbackVideoDraftTimeStateRef.current ??
              playbackVideoCurrentTimeStateRef.current ??
              previous.currentTime ??
              0
          ),
          paused: playbackVideoPausedStateRef.current,
          playbackRate: playbackVideoRateStateRef.current || previous.playbackRate || 1
        };
        playbackVideoCarryoverPendingRef.current = true;
      }

      playbackVideoRef.current = node;
      setPlaybackVideoElement(node);
      if (!node) {
        return;
      }

      playbackVideoInitializedRef.current = false;
      playbackVideoLastSyncSignatureRef.current = null;
      playbackVideoLastFrameSyncRef.current = null;
      playbackVideoLastSyncWallRef.current = 0;
      playbackVideoLastSeekWallRef.current = 0;

      const carryover = playbackVideoCarryoverRef.current;
      if (!carryover) {
        refreshPlaybackVideoState();
        return;
      }

      const applyCarryover = () => {
        const durationSec = Number.isFinite(node.duration) ? Math.max(node.duration, 0) : 0;
        const targetTimeSec =
          durationSec > 0
            ? Math.min(carryover.currentTimeSec, durationSec)
            : carryover.currentTimeSec;
        if (Math.abs((node.currentTime || 0) - targetTimeSec) > 0.05) {
          node.currentTime = targetTimeSec;
        }
        node.playbackRate = carryover.playbackRate;
        refreshPlaybackVideoState();
        const shouldRemainPaused = playbackStatusRef.current?.paused ?? carryover.paused;
        if (shouldRemainPaused) {
          node.pause();
        } else {
          void node.play().catch(() => {
            refreshPlaybackVideoState();
          });
        }
        playbackVideoCarryoverRef.current = null;
        playbackVideoCarryoverPendingRef.current = false;
      };

      if (node.readyState >= HTMLMediaElement.HAVE_METADATA) {
        applyCarryover();
        return;
      }

      const handleLoadedMetadata = () => {
        node.removeEventListener("loadedmetadata", handleLoadedMetadata);
        applyCarryover();
      };
      node.addEventListener("loadedmetadata", handleLoadedMetadata);
    },
    [
      refreshPlaybackVideoState
    ]
  );

  const pausePlaybackVideo = useCallback(() => {
    const video = playbackVideoRef.current;
    if (!video) {
      return;
    }
    playbackVideoPlayPromiseRef.current = null;
    video.pause();
    refreshPlaybackVideoState();
  }, [refreshPlaybackVideoState]);

  const requestPlaybackVideoPlay = useCallback(async (): Promise<boolean> => {
    const video = playbackVideoRef.current;
    if (!video) {
      return false;
    }

    try {
      const playPromise = video.play();
      if (!playPromise) {
        refreshPlaybackVideoState();
        return true;
      }
      playbackVideoPlayPromiseRef.current = playPromise;
      await playPromise;
      if (playbackVideoPlayPromiseRef.current === playPromise) {
        playbackVideoPlayPromiseRef.current = null;
      }
      refreshPlaybackVideoState();
      return true;
    } catch (error) {
      if (playbackVideoPlayPromiseRef.current) {
        playbackVideoPlayPromiseRef.current = null;
      }
      if (!isBenignPlaybackVideoPlayError(error)) {
        setPlaybackError(error instanceof Error ? error.message : "Playback video failed to start.");
      }
      refreshPlaybackVideoState();
      return false;
    }
  }, [refreshPlaybackVideoState]);

  const seekPlaybackVideo = useCallback(
    async (targetTimeSec: number) => {
      const video = playbackVideoRef.current;
      if (!video) {
        return;
      }

      const clampedTargetSec = clampNumber(targetTimeSec, 0, playbackVideoDurationSec || targetTimeSec);
      if (Math.abs((video.currentTime || 0) - clampedTargetSec) <= 0.01) {
        refreshPlaybackVideoState();
        return;
      }

      const settlePromise = waitForPlaybackVideoSeek(video, clampedTargetSec);
      video.currentTime = clampedTargetSec;
      refreshPlaybackVideoState();
      await settlePromise;
      refreshPlaybackVideoState();
    },
    [playbackVideoDurationSec, refreshPlaybackVideoState]
  );

  const updatePlaybackVideoDraft = useCallback(
    (nextTimeSec: number) => {
      const video = playbackVideoRef.current;
      if (video && !playbackVideoScrubbingRef.current) {
        playbackVideoScrubbingRef.current = true;
        playbackVideoScrubResumeRef.current = !video.paused && !video.ended;
        if (playbackVideoScrubResumeRef.current) {
          pausePlaybackVideo();
        }
      }

      const clampedTargetSec = clampNumber(nextTimeSec, 0, playbackVideoDurationSec || nextTimeSec);
      setPlaybackVideoDraftTimeSec(clampedTargetSec);

      if (!video || !playbackVideoReady) {
        return;
      }

      if (Math.abs((video.currentTime || 0) - clampedTargetSec) > 0.01) {
        video.currentTime = clampedTargetSec;
      }
      refreshPlaybackVideoState();
    },
    [pausePlaybackVideo, playbackVideoDurationSec, playbackVideoReady, refreshPlaybackVideoState]
  );

  const playbackShowcase =
    playbackShowcaseEnabled && consoleMode === "playback" && playbackPresentation === "showcase";
  const playbackCinemaLayout = consoleMode === "playback" && !playbackShowcase;
  const playbackCloudOptions = useMemo<PlaybackCloudOptions>(
    () => ({
      enabled: true,
      decayTimeSec: playbackDecaySec,
      ...playbackCloudProfiles[playbackCloudProfile],
      ...(playbackShowcase && playbackCloudProfile === "quality"
        ? {
            maxInputPoints: 9800,
            maxAccumulatedPoints: 620_000,
            voxelSize: 0.13
          }
        : {})
    }),
    [playbackCloudProfile, playbackDecaySec, playbackShowcase]
  );
  const rosState = useNeuralMappingRos(wsUrl, {
    playbackCloud: playbackCloudOptions,
    playbackTargetStampMs:
      consoleMode === "playback" ? playbackStatus?.currentPose?.stampMs ?? null : null
  });
  hifiManifestCameraRef.current = manifest?.camera;
  const effectiveManifest = useMemo(
    () => resolveGaussianVariant(manifest, gaussianVariant),
    [manifest, gaussianVariant]
  );
  const gaussianVariants = useMemo(
    () => getGaussianVariants(manifest),
    [manifest]
  );
  const activeGaussianVariant =
    gaussianVariant ??
    effectiveManifest?.gaussian?.variant ??
    manifest?.gaussian?.variant ??
    gaussianVariants[0]?.[0] ??
    "quality";
  const occupancyCloudAssetState = manifest?.assets?.rawPointCloud ?? "loading";
  const occupancyCloudLabel =
    consoleMode === "playback"
      ? "Accumulated Map"
      : occupancyCloudAssetState === "fallback"
        ? "Occupancy Cloud"
        : "Static Cloud";
  const liveCloudLabel = consoleMode === "playback" ? "Current Scan" : "ROS Cloud";
  const viewerPresentationMode: PresentationMode = playbackShowcase ? "showcase" : "default";
  const effectiveFollowRobot = followRobot;

  const viewerMode: ViewerMode =
    consoleMode === "gs" || consoleMode === "hifi" || consoleMode === "compare"
      ? "gs"
      : consoleMode === "playback"
        ? "playback"
        : "live";
  const hifiNativeRendererReady = Boolean(hifiWindowStatus?.ready || hifiStatus?.ready);
  const hifiBrowserFallbackActive = consoleMode === "hifi" && !hifiNativeRendererReady;
  const modeLabel =
    consoleMode === "compare"
      ? "Compare Review"
      : consoleMode === "hifi"
        ? "High Fidelity"
      : consoleMode === "gs"
        ? "GS Layer"
        : consoleMode === "playback"
          ? "Playback Mapping"
          : "Live Mapping";
  const mainViewSource =
    consoleMode === "hifi"
      ? hifiBrowserFallbackActive
        ? "Browser GS fallback"
        : "Native render"
      : consoleMode === "playback"
      ? [
          layers.gaussian ? "Gaussian" : null,
          layers.sdfMesh ? "SDF mesh" : null,
          layers.liveCloud ? "Current playback scan" : null,
          layers.occupancyCloud ? "Accumulated playback map" : null
        ].filter(Boolean).join(" + ") || "Playback geometry / overlays"
      : viewerMode === "gs"
      ? layers.sdfMesh
        ? "Gaussian + mesh"
        : "Gaussian"
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
    rosState.streamProfile,
    hifiBrowserFallbackActive
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
    gsControlMode,
    hifiBrowserFallbackActive
  );
  const playbackVideoDescriptor = useMemo<PlaybackVideoDescriptor | null>(
    () => playbackStatus?.video ?? null,
    [playbackStatus]
  );
  const playbackVideoSourceUrl = useMemo(
    () => resolvePlaybackMediaUrl(playbackApiUrl, playbackVideoDescriptor?.url),
    [playbackApiUrl, playbackVideoDescriptor?.url]
  );
  const playbackVideoSyncConfig = useMemo(
    () => ({
      fps: playbackVideoDescriptor?.fps ?? fallbackPlaybackVideoSyncConfig.fps,
      speed: playbackVideoDescriptor?.speed ?? fallbackPlaybackVideoSyncConfig.speed,
      startOffsetSec: playbackVideoDescriptor?.startOffsetSec ?? fallbackPlaybackVideoSyncConfig.startOffsetSec,
      endOffsetSec: playbackVideoDescriptor?.endOffsetSec ?? fallbackPlaybackVideoSyncConfig.endOffsetSec
    }),
    [
      playbackVideoDescriptor?.endOffsetSec,
      playbackVideoDescriptor?.fps,
      playbackVideoDescriptor?.speed,
      playbackVideoDescriptor?.startOffsetSec
    ]
  );
  const playbackVideoToBagScale = useMemo(
    () => computePlaybackVideoToBagScale(playbackVideoDurationSec, playbackStatus?.durationSec ?? 0),
    [playbackStatus?.durationSec, playbackVideoDurationSec]
  );
  const effectivePlaybackVideoTimeSec =
    playbackVideoDraftTimeSec ?? playbackVideoPresentedTimeSec ?? playbackVideoCurrentTimeSec;
  const effectivePlaybackOffset = videoTimeToPlaybackOffsetSec(
    effectivePlaybackVideoTimeSec,
    playbackVideoDurationSec,
    playbackStatus?.durationSec ?? 0,
    playbackVideoSyncMap
  );
  const playbackVisualVideoFrameIndex = playbackVideoSyncFrameIndex ?? null;
  const playbackVideoDerivedPlaybackFrameIndex =
    playbackVideoSyncMap &&
    playbackVisualVideoFrameIndex !== null &&
    playbackVisualVideoFrameIndex >= 0 &&
    playbackVisualVideoFrameIndex < playbackVideoSyncMap.frameIndices.length
      ? playbackVideoSyncMap.frameIndices[playbackVisualVideoFrameIndex] ?? null
      : null;
  const playbackVisualPlaybackFrameIndex =
    playbackStatus?.frameIndex ?? playbackVideoDerivedPlaybackFrameIndex ?? null;
  const playbackPoseTrackPose =
    playbackPoseTrack &&
    playbackVisualPlaybackFrameIndex !== null &&
    playbackVisualPlaybackFrameIndex >= 0 &&
    playbackVisualPlaybackFrameIndex < playbackPoseTrack.poses.length
      ? playbackPoseTrack.poses[playbackVisualPlaybackFrameIndex] ?? null
      : null;
  const playbackTargetStampMs =
    consoleMode === "playback"
      ? playbackStatus?.currentPose?.stampMs ?? playbackPoseTrackPose?.stampMs ?? null
      : null;
  const playbackOverlayScan = playbackOverlayScanPointCloud ?? rosState.playbackScanPointCloud;
  const playbackTrajectorySource = useMemo(
    () =>
      consoleMode === "playback" && playbackPoseTrack && playbackVisualPlaybackFrameIndex !== null
        ? buildPlaybackTrajectoryFromPoseTrack(
            playbackPoseTrack.poses,
            playbackVisualPlaybackFrameIndex,
            playbackShowcase ? 4096 : 3072
          )
        : playbackOverlayTrajectory ?? rosState.trajectory,
    [
      consoleMode,
      playbackOverlayTrajectory,
      playbackPoseTrack,
      playbackShowcase,
      playbackVisualPlaybackFrameIndex,
      rosState.trajectory
    ]
  );
  const playbackOverlayRobotPose = playbackOverlayTrajectory?.at(-1) ?? null;
  useEffect(() => {
    if (consoleMode !== "playback") {
      setPlaybackResolvedMapPointCloud(null);
      return;
    }

    const nextMap =
      playbackOverlayAccumulatedPointCloud ??
      rosState.playbackAccumulatedPointCloud ??
      null;
    if (!nextMap || nextMap.renderedPointCount < 1) {
      return;
    }
    setPlaybackResolvedMapPointCloud(nextMap);
  }, [
    consoleMode,
    playbackOverlayAccumulatedPointCloud,
    rosState.playbackAccumulatedPointCloud
  ]);
  const hasLiveCloud = Boolean(
    rosState.livePointCloud ||
      playbackResolvedMapPointCloud ||
      playbackOverlayScan
  );
  const activeMapPointCloud =
    consoleMode === "playback"
      ? playbackResolvedMapPointCloud ??
        playbackOverlayScan ??
        rosState.playbackScanPointCloud ??
        rosState.livePointCloud
      : rosState.livePointCloud;
  const playbackTopDownMapPointCloud = useMemo(
    () =>
      consoleMode === "playback"
        ? pickPlaybackAccumulatedMapPointCloudForTarget(
            playbackTargetStampMs,
            playbackResolvedMapPointCloud,
            playbackOverlayAccumulatedPointCloud,
            playbackOverlayScan,
            rosState.playbackScanPointCloud,
            rosState.livePointCloud
          )
        : activeMapPointCloud,
    [
      activeMapPointCloud,
      consoleMode,
      playbackOverlayAccumulatedPointCloud,
      playbackOverlayScan,
      playbackResolvedMapPointCloud,
      playbackTargetStampMs,
      rosState.livePointCloud,
      rosState.playbackScanPointCloud
    ]
  );
  const playbackRobotPose =
    consoleMode === "playback"
      ? playbackStatus?.currentPose ??
        playbackPoseTrackPose ??
        rosState.robotPose ??
        playbackOverlayRobotPose ??
        null
      : null;
  const playbackDisplayTrajectory = useMemo(() => {
    if (consoleMode !== "playback") {
      return rosState.trajectory;
    }
    const maxStampMs =
      playbackRobotPose?.stampMs ??
      (playbackCompareMode === "live" ? null : playbackStatus?.currentPose?.stampMs) ??
      null;
    if (!maxStampMs) {
      return playbackTrajectorySource;
    }
    const filtered = playbackTrajectorySource.filter((pose) => (pose.stampMs ?? 0) <= maxStampMs + 1);
    return filtered.length > 0 ? filtered : playbackTrajectorySource;
  }, [
    consoleMode,
    playbackCompareMode,
    playbackRobotPose?.stampMs,
    playbackStatus?.currentPose?.stampMs,
    playbackTrajectorySource,
    rosState.trajectory
  ]);
  const displayRobotPose =
    consoleMode === "playback"
      ? playbackRobotPose ?? rosState.robotPose
      : navPreviewPose ?? rosState.robotPose;
  const displayTrajectory =
    consoleMode === "playback"
      ? playbackDisplayTrajectory
      : navPreviewActive || navPreviewTrajectory.length > 1
        ? navPreviewTrajectory
        : playbackDisplayTrajectory;
  const playbackTimelineDisabled = !playbackStatus || !playbackVideoReady;
  const visiblePlaybackError =
    playbackError ??
    (consoleMode === "playback" && playbackStatus && !playbackVideoDescriptor
      ? "Offline playback video is not configured for this bag."
      : null);
  const playbackRgbMjpegUrl = useMemo(
    () => buildPlaybackVideoUrl(playbackApiUrl, "/frame.mjpeg"),
    [playbackApiUrl]
  );
  const hifiNativeVideoStreamUrl = useMemo(
    () => buildPlaybackVideoUrl(hifiStreamUrlBase, "/frame.mjpeg"),
    [hifiStreamUrlBase]
  );
  const hifiNativeFrameUrl = useMemo(
    () => buildPlaybackVideoUrl(hifiStreamUrlBase, "/frame.jpg"),
    [hifiStreamUrlBase]
  );
  const hifiBridgeFrameUrl = useMemo(
    () => buildPlaybackVideoUrl(hifiApiUrl, "/frame.jpg"),
    [hifiApiUrl]
  );
  const hifiBridgeMjpegUrl = useMemo(
    () => buildPlaybackVideoUrl(hifiApiUrl, "/frame.mjpeg"),
    [hifiApiUrl]
  );
  const hifiBridgePreviewFrameUrl = useMemo(
    () => buildPlaybackVideoUrl(hifiApiUrl, "/frame.preview.jpg"),
    [hifiApiUrl]
  );
  const mediaSessionKey = useMemo(
    () =>
      [
        consoleMode,
        playbackPresentation,
        playbackCompareMode,
        viewerMode,
        activeGaussianVariant,
        playbackApiUrl,
        hifiApiUrl,
        hifiStreamUrlBase
      ].join(":"),
    [
      activeGaussianVariant,
      consoleMode,
      hifiApiUrl,
      hifiStreamUrlBase,
      playbackApiUrl,
      playbackCompareMode,
      playbackPresentation,
      viewerMode
    ]
  );
  const hifiActive = consoleMode === "hifi" || (consoleMode === "playback" && playbackCompareMode === "live");
  const hifiUsingNativeWindow = Boolean(hifiWindowStatus?.ready && !hifiStreamFailed);
  const hifiVideoStreamUrl = hifiUsingNativeWindow ? hifiNativeVideoStreamUrl : buildPlaybackVideoUrl(hifiApiUrl, "/frame.mjpeg");
  const hifiFrameUrl = hifiUsingNativeWindow ? hifiNativeFrameUrl : hifiBridgeFrameUrl;
  const hifiPreviewFrameUrl = hifiUsingNativeWindow ? hifiNativeFrameUrl : hifiBridgePreviewFrameUrl;
  const playbackPoseForCompare = playbackRobotPose ?? rosState.robotPose ?? null;
  const playbackGsComparePose = useMemo(
    () => transformRobotPoseToPlaybackCameraPose(playbackPoseForCompare, manifest?.camera),
    [manifest?.camera, playbackPoseForCompare]
  );
  const playbackCompareNote = useMemo(() => {
    if (playbackCompareMode === "video") {
      return "Offline RGB · GS-SDF clip is the master timeline; 2D map and Main 3D follow the same bag time.";
    }
    if (playbackCompareMode === "webgs") {
      return "Offline video still drives playback time; Playback RGB, Spark GS, and Main 3D stay locked to the same bag pose.";
    }
    if (!playbackStatus) {
      return "Waiting for playback.";
    }
    if (!playbackGsComparePose) {
      return "Waiting for pose.";
    }
    if (!hifiNativeRendererReady) {
      return "HiFi GS renderer offline.";
    }
    return "HiFi MJPEG — pose pushed over HTTP; expect extra latency vs Playback RGB.";
  }, [
    playbackCompareMode,
    hifiNativeRendererReady,
    playbackGsComparePose,
    playbackStatus
  ]);
  const renderPlaybackMasterVideo = (className = "compare-video"): ReactNode => (
    <video
      ref={setPlaybackVideoNode}
      className={className}
      src={playbackVideoSourceUrl ?? undefined}
      muted
      playsInline
      preload="auto"
      onLoadStart={() => {
        setPlaybackVideoReady(false);
        setPlaybackError(null);
        refreshPlaybackVideoState();
      }}
      onLoadedMetadata={() => {
        playbackVideoInitializedRef.current = false;
        playbackVideoLastSyncSignatureRef.current = null;
        setPlaybackError(null);
        refreshPlaybackVideoState();
      }}
      onLoadedData={() => {
        setPlaybackError(null);
        refreshPlaybackVideoState();
      }}
      onCanPlay={() => {
        setPlaybackError(null);
        refreshPlaybackVideoState();
      }}
      onDurationChange={refreshPlaybackVideoState}
      onTimeUpdate={refreshPlaybackVideoState}
      onSeeked={refreshPlaybackVideoState}
      onPlay={refreshPlaybackVideoState}
      onPause={refreshPlaybackVideoState}
      onRateChange={refreshPlaybackVideoState}
      onError={() => {
        setPlaybackVideoReady(false);
        setPlaybackError("Offline playback video failed to load.");
      }}
      onEnded={() => {
        refreshPlaybackVideoState();
        void syncPlaybackToMasterVideo(true);
      }}
    />
  );

  useEffect(() => {
    if (consoleMode !== "playback") {
      setPlaybackError(null);
      return;
    }

    let disposed = false;
    const clearTimerId = window.setTimeout(() => {
      if (!disposed && playbackStatus && playbackVideoReady) {
        setPlaybackError(null);
      }
    }, 0);

    return () => {
      disposed = true;
      window.clearTimeout(clearTimerId);
    };
  }, [consoleMode, playbackStatus, playbackVideoReady]);

  useEffect(() => {
    setSceneError(null);
    setPlaybackError(null);
    setHifiError(null);
    setHifiStreamFailed(false);
  }, [consoleMode, playbackPresentation, playbackCompareMode]);

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
        const candidates = buildControlUrlCandidates(playbackApiUrl, "/__playback_control", 8765);
        let lastError: unknown = null;
        for (const candidate of candidates) {
          try {
            const response = await fetch(`${candidate}/status`, { cache: "no-store" });
            if (!response.ok) {
              throw new Error(`Playback control returned ${response.status}`);
            }
            const status = (await response.json()) as PlaybackStatus;
            playbackStatusFailureCountRef.current = 0;
            hasReceivedStatus = true;
            if (!disposed) {
              if (candidate !== playbackApiUrl) {
                setPlaybackApiUrl(candidate);
              }
              commitPlaybackStatus(status);
              setPlaybackError(null);
            }
            return;
          } catch (error) {
            lastError = error;
          }
        }
        playbackStatusFailureCountRef.current += 1;
        if (!disposed && playbackStatusFailureCountRef.current >= 3) {
          setPlaybackError(humanizeNetworkError(lastError, "playback control"));
        }
      } finally {
        playbackStatusRequestInFlightRef.current = false;
      }
    };

    const warmupTimerId = window.setTimeout(() => {
      if (!disposed && !hasReceivedStatus && !playbackStatusRef.current) {
        setPlaybackError("Playback control is still starting.");
      }
    }, 2000);

    void loadStatus();
    const intervalId = window.setInterval(loadStatus, 120);
    return () => {
      disposed = true;
      window.clearTimeout(warmupTimerId);
      window.clearInterval(intervalId);
      playbackStatusRequestInFlightRef.current = false;
      playbackStatusFailureCountRef.current = 0;
    };
  }, [consoleMode, playbackApiUrl]);

  useEffect(() => {
    playbackVideoInitializedRef.current = false;
    playbackVideoLastSyncSignatureRef.current = null;
    playbackVideoLastFrameSyncRef.current = null;
    playbackVideoLastSyncWallRef.current = 0;
    playbackVideoInitializingRef.current = false;
    setPlaybackVideoDraftTimeSec(null);
    setPlaybackVideoPresentedTimeSec(null);
    setPlaybackVideoSyncFrameIndex(null);
    playbackOverlayDesiredRef.current = null;
    playbackOverlayLoadedKeyRef.current = null;
    playbackOverlayLastRequestedFrameRef.current = null;
    playbackOverlayLastRequestWallRef.current = 0;
    playbackOverlayFetchInFlightRef.current = false;
    playbackOverlayAbortRef.current?.abort();
    playbackOverlayAbortRef.current = null;
    setPlaybackOverlayTrajectory(null);
    setPlaybackOverlayScanPointCloud(null);
    setPlaybackOverlayFrameIndex(null);
    playbackOverlayAccumulationRef.current = {
      voxels: new Map(),
      order: [],
      lastFrameIndex: null
    };
    setPlaybackOverlayAccumulatedPointCloud(null);
    setPlaybackResolvedMapPointCloud(null);
    setPlaybackError(null);
    refreshPlaybackVideoState();
  }, [playbackVideoSourceUrl, refreshPlaybackVideoState]);

  useEffect(() => {
    playbackStreamSourcePoseRef.current = playbackPoseForCompare ?? rosState.robotPose ?? null;
  }, [playbackPoseForCompare, rosState.robotPose]);

  useEffect(() => {
    if (consoleMode !== "playback" || !playbackVideoSourceUrl) {
      setPlaybackVideoSyncMap(null);
      return;
    }

    let disposed = false;
    const load = async () => {
      const candidates = buildControlUrlCandidates(playbackApiUrl, "/__playback_control", 8765);
      for (const candidate of candidates) {
        try {
          const query = new URLSearchParams({
            fps: String(playbackVideoSyncConfig.fps),
            speed: String(playbackVideoSyncConfig.speed),
            startOffsetSec: String(playbackVideoSyncConfig.startOffsetSec),
            endOffsetSec: String(playbackVideoSyncConfig.endOffsetSec)
          });
          const response = await fetch(`${candidate}/video_sync?${query.toString()}`, {
            cache: "no-store"
          });
          if (!response.ok) {
            throw new Error(`Playback video sync returned ${response.status}`);
          }
          const payload = (await response.json()) as PlaybackVideoSyncMap;
          if (!disposed) {
            setPlaybackVideoSyncMap(payload);
          }
          return;
        } catch {
          continue;
        }
      }

      if (!disposed) {
        setPlaybackVideoSyncMap(null);
      }
    };

    void load();
    return () => {
      disposed = true;
    };
  }, [
    consoleMode,
    playbackApiUrl,
    playbackVideoSourceUrl,
    playbackVideoSyncConfig.endOffsetSec,
    playbackVideoSyncConfig.fps,
    playbackVideoSyncConfig.speed,
    playbackVideoSyncConfig.startOffsetSec
  ]);

  useEffect(() => {
    if (consoleMode !== "playback") {
      setPlaybackPoseTrack(null);
      return;
    }

    let disposed = false;
    const load = async () => {
      const candidates = buildControlUrlCandidates(playbackApiUrl, "/__playback_control", 8765);
      for (const candidate of candidates) {
        try {
          const response = await fetch(`${candidate}/pose_track.json`, {
            cache: "no-store"
          });
          if (!response.ok) {
            throw new Error(`Playback pose track returned ${response.status}`);
          }
          const payload = (await response.json()) as PlaybackPoseTrack;
          if (!disposed) {
            setPlaybackPoseTrack(payload);
          }
          return;
        } catch {
          continue;
        }
      }

      if (!disposed) {
        setPlaybackPoseTrack(null);
      }
    };

    void load();
    return () => {
      disposed = true;
    };
  }, [consoleMode, playbackApiUrl]);

  const pumpPlaybackOverlay = useCallback(async () => {
    if (consoleMode !== "playback" || playbackOverlayFetchInFlightRef.current) {
      return;
    }

    while (true) {
      const desired = playbackOverlayDesiredRef.current;
      if (!desired) {
        return;
      }

      const desiredKey = `${desired.frameIndex}:${desired.tailSec.toFixed(3)}`;
      if (playbackOverlayLoadedKeyRef.current === desiredKey) {
        return;
      }

      playbackOverlayFetchInFlightRef.current = true;
      const controller = new AbortController();
      playbackOverlayAbortRef.current = controller;
      let loaded = false;

      const candidates = buildControlUrlCandidates(playbackApiUrl, "/__playback_control", 8765);
      for (const candidate of candidates) {
        try {
          const overlayScanMaxPoints = playbackShowcase ? "9000" : "7200";
          const overlayScanHistoryFrames = playbackShowcase ? "2" : "1";
          const overlayMapMaxPoints = playbackShowcase ? "6000" : "3000";
          const query = new URLSearchParams({
            frameIndex: String(desired.frameIndex),
            trajectoryMaxPoints: "1",
            trajectoryTailSec: "0",
            scanMaxPoints: overlayScanMaxPoints,
            scanHistoryFrames: overlayScanHistoryFrames,
            mapMaxPoints: overlayMapMaxPoints
          });
          const response = await fetch(`${candidate}/overlay.json?${query.toString()}`, {
            cache: "no-store",
            signal: controller.signal
          });
          if (!response.ok) {
            throw new Error(`Playback overlay returned ${response.status}`);
          }
          const payload = (await response.json()) as PlaybackOverlayState;
          if (controller.signal.aborted) {
            playbackOverlayFetchInFlightRef.current = false;
            return;
          }
          if (candidate !== playbackApiUrl) {
            setPlaybackApiUrl(candidate);
          }
          setPlaybackOverlayTrajectory(payload.trajectory ?? []);
          setPlaybackOverlayScanPointCloud(decodePlaybackOverlayScan(payload.scan));
          setPlaybackOverlayAccumulatedPointCloud(
            payload.map ? decodePlaybackOverlayScan(payload.map) : null
          );
          setPlaybackOverlayFrameIndex(payload.frameIndex ?? desired.frameIndex);
          playbackOverlayLoadedKeyRef.current = desiredKey;
          loaded = true;
          break;
        } catch (error) {
          if (controller.signal.aborted || (error instanceof DOMException && error.name === "AbortError")) {
            playbackOverlayFetchInFlightRef.current = false;
            return;
          }
          continue;
        }
      }

      playbackOverlayFetchInFlightRef.current = false;
      if (playbackOverlayAbortRef.current === controller) {
        playbackOverlayAbortRef.current = null;
      }

      if (!loaded) {
        playbackOverlayLoadedKeyRef.current = null;
        return;
      }

      const latest = playbackOverlayDesiredRef.current;
      const latestKey = latest ? `${latest.frameIndex}:${latest.tailSec.toFixed(3)}` : null;
      if (!latestKey || latestKey === desiredKey) {
        return;
      }
    }
  }, [consoleMode, playbackApiUrl, playbackShowcase]);

  useEffect(() => {
    if (consoleMode !== "playback") {
      playbackOverlayDesiredRef.current = null;
      playbackOverlayLoadedKeyRef.current = null;
      playbackOverlayLastRequestedFrameRef.current = null;
      playbackOverlayLastRequestWallRef.current = 0;
      playbackOverlayFetchInFlightRef.current = false;
      playbackOverlayAbortRef.current?.abort();
      playbackOverlayAbortRef.current = null;
      setPlaybackOverlayTrajectory(null);
      setPlaybackOverlayScanPointCloud(null);
      setPlaybackOverlayFrameIndex(null);
      return;
    }

    const frameIndex = playbackVisualPlaybackFrameIndex;
    if (frameIndex === undefined || frameIndex === null || frameIndex < 0) {
      return;
    }

    const now = performance.now();
    const lastRequestedFrame = playbackOverlayLastRequestedFrameRef.current;
    const lastRequestedWall = playbackOverlayLastRequestWallRef.current;
    const minFrameDelta = playbackShowcase ? 2 : 1;
    const minRequestGapMs = playbackShowcase ? 95 : 120;
    const playing = !playbackVideoPausedStateRef.current;
    if (
      playing &&
      lastRequestedFrame !== null &&
      frameIndex >= lastRequestedFrame &&
      frameIndex - lastRequestedFrame < minFrameDelta &&
      now - lastRequestedWall < minRequestGapMs
    ) {
      return;
    }

    playbackOverlayLastRequestedFrameRef.current = frameIndex;
    playbackOverlayLastRequestWallRef.current = now;
    playbackOverlayDesiredRef.current = {
      frameIndex,
      tailSec: 0
    };
    void pumpPlaybackOverlay();
  }, [consoleMode, playbackShowcase, playbackVisualPlaybackFrameIndex, pumpPlaybackOverlay]);

  useEffect(() => {
    playbackVideoLastSyncSignatureRef.current = null;
    playbackVideoLastFrameSyncRef.current = null;
    playbackVideoLastSyncWallRef.current = 0;
    playbackVideoLastSeekWallRef.current = 0;
    playbackOverlayLastRequestedFrameRef.current = null;
    playbackOverlayLastRequestWallRef.current = 0;
  }, [playbackVideoSyncMap]);

  useEffect(() => {
    if (consoleMode !== "playback" || !playbackVideoReady) {
      setPlaybackVideoSyncFrameIndex(null);
      return;
    }

    const video = playbackVideoRef.current;
    if (!video) {
      setPlaybackVideoSyncFrameIndex(null);
      return;
    }

    let disposed = false;
    let callbackId = 0;
    let intervalId = 0;

    const stopLoop = () => {
      const frameVideo = video as HTMLVideoElement & {
        cancelVideoFrameCallback?(handle: number): void;
      };
      if (callbackId) {
        frameVideo.cancelVideoFrameCallback?.(callbackId);
        callbackId = 0;
      }
      if (intervalId) {
        window.clearInterval(intervalId);
        intervalId = 0;
      }
    };

    const updateFrameIndex = (timeSec?: number) => {
      if (disposed) {
        return;
      }
      const durationSec = Number.isFinite(video.duration) ? Math.max(video.duration, 0) : 0;
      const resolvedTimeSec =
        typeof timeSec === "number" && Number.isFinite(timeSec)
          ? Math.max(0, timeSec)
          : Math.max(0, video.currentTime || 0);
      const clampedPresentedTimeSec =
        video.ended && durationSec > 0 ? durationSec : resolvedTimeSec;
      setPlaybackVideoPresentedTimeSec((current) =>
        current !== null && Math.abs(current - clampedPresentedTimeSec) <= 1e-3
          ? current
          : clampedPresentedTimeSec
      );
      const nextFrameIndex =
        video.ended && playbackVideoSyncMap?.videoFrameCount
          ? Math.max(playbackVideoSyncMap.videoFrameCount - 1, 0)
          : resolvePlaybackFrameIndexFromVideoTime(clampedPresentedTimeSec, playbackVideoSyncMap);
      setPlaybackVideoSyncFrameIndex((current) =>
        current === nextFrameIndex ? current : nextFrameIndex
      );
    };

    const startLoop = () => {
      stopLoop();
      updateFrameIndex();
      if (video.paused || video.ended) {
        return;
      }
      if ("requestVideoFrameCallback" in video) {
        const frameVideo = video as HTMLVideoElement & {
          requestVideoFrameCallback(
            callback: (now: number, metadata: { mediaTime?: number }) => void
          ): number;
        };
        const tick = (_now: number, metadata: { mediaTime?: number }) => {
          updateFrameIndex(metadata?.mediaTime);
          callbackId = frameVideo.requestVideoFrameCallback(tick);
        };
        callbackId = frameVideo.requestVideoFrameCallback(tick);
        return;
      }
      intervalId = window.setInterval(updateFrameIndex, 75);
    };

    const handlePassiveUpdate = () => {
      updateFrameIndex();
      if (!video.paused && !video.ended) {
        startLoop();
      }
    };

    updateFrameIndex();
    startLoop();
    video.addEventListener("loadedmetadata", handlePassiveUpdate);
    video.addEventListener("loadeddata", handlePassiveUpdate);
    video.addEventListener("seeked", handlePassiveUpdate);
    video.addEventListener("timeupdate", handlePassiveUpdate);
    video.addEventListener("play", startLoop);
    video.addEventListener("pause", handlePassiveUpdate);
    video.addEventListener("ratechange", handlePassiveUpdate);
    video.addEventListener("ended", handlePassiveUpdate);

    return () => {
      disposed = true;
      stopLoop();
      video.removeEventListener("loadedmetadata", handlePassiveUpdate);
      video.removeEventListener("loadeddata", handlePassiveUpdate);
      video.removeEventListener("seeked", handlePassiveUpdate);
      video.removeEventListener("timeupdate", handlePassiveUpdate);
      video.removeEventListener("play", startLoop);
      video.removeEventListener("pause", handlePassiveUpdate);
      video.removeEventListener("ratechange", handlePassiveUpdate);
      video.removeEventListener("ended", handlePassiveUpdate);
    };
  }, [consoleMode, playbackVideoElement, playbackVideoReady, playbackVideoSyncMap]);

  useEffect(() => {
    if (consoleMode !== "playback" || playbackShowcase || playbackCompareMode !== "live") {
      return;
    }

    let disposed = false;
    let timeoutId = 0;
    const pump = () => {
      if (disposed) {
        return;
      }
      const pose = playbackStreamSourcePoseRef.current;
      const paused = playbackStatusRef.current?.paused ?? false;
      if (!pose || !hifiApiUrl) {
        timeoutId = window.setTimeout(pump, 200);
        return;
      }
      if (!playbackHiFiPoseBusyRef.current) {
        const cam = transformRobotPoseToPlaybackCameraPose(pose, hifiManifestCameraRef.current);
        if (cam) {
          playbackHiFiPoseBusyRef.current = true;
          void fetch(`${hifiApiUrl}/camera_pose`, {
            method: "POST",
            headers: {
              "Content-Type": "application/json"
            },
            body: JSON.stringify({
              position: cam.position,
              orientation: cam.orientation
            })
          })
            .catch(() => {
              /* HiFi may be offline */
            })
            .finally(() => {
              playbackHiFiPoseBusyRef.current = false;
            });
        }
      }
      timeoutId = window.setTimeout(pump, paused ? 400 : 33);
    };

    pump();
    return () => {
      disposed = true;
      window.clearTimeout(timeoutId);
    };
  }, [consoleMode, playbackCompareMode, playbackShowcase, hifiApiUrl]);

  useEffect(() => {
    if (consoleMode !== "playback" || playbackShowcase || playbackCompareMode !== "webgs") {
      return;
    }

    const viewer = viewerRef.current;
    if (!viewer || !playbackGsComparePose) {
      return;
    }

    viewer.setCameraPose(
      playbackGsComparePose,
      cameraTargetFromPose(playbackGsComparePose, 2.5)
    );
  }, [consoleMode, playbackCompareMode, playbackShowcase, playbackGsComparePose]);

  useEffect(() => {
    const usesWebGs =
      consoleMode === "playback" && !playbackShowcase && playbackCompareMode === "webgs";
    viewerRef.current?.setPlaybackCompareUsesWebGs(usesWebGs);
  }, [consoleMode, playbackCompareMode, playbackShowcase]);

  const sendPlaybackControl = async (payload: {
    rate?: number;
    paused?: boolean;
    offsetSec?: number;
  }, options?: {
    background?: boolean;
  }): Promise<boolean> => {
    if (!options?.background) {
      setPlaybackBusy(true);
    }
      try {
        const candidates = buildControlUrlCandidates(playbackApiUrl, "/__playback_control", 8765);
        let lastError: unknown = null;
        for (const candidate of candidates) {
          try {
            const response = await fetch(`${candidate}/control`, {
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
            if (candidate !== playbackApiUrl) {
              setPlaybackApiUrl(candidate);
            }
            commitPlaybackStatus(status);
            setPlaybackVideoDraftTimeSec(null);
            setPlaybackError(null);
            return true;
          } catch (error) {
            lastError = error;
          }
        }
        throw lastError instanceof Error ? lastError : new Error("Playback control unavailable.");
    } catch (error) {
      setPlaybackError(humanizeNetworkError(error, "playback control"));
      return false;
    } finally {
      if (!options?.background) {
        setPlaybackBusy(false);
      }
    }
  };

  const syncPlaybackToMasterVideo = useCallback(
    async (force = false) => {
      if (consoleMode !== "playback") {
        return;
      }

      const video = playbackVideoRef.current;
      const latestStatus = playbackStatusRef.current ?? playbackStatus;
      const playbackDurationSec = latestStatus?.durationSec ?? 0;
      if (!video || !playbackVideoReady || playbackDurationSec <= 0) {
        return;
      }
      if (playbackVideoCarryoverPendingRef.current && !force) {
        return;
      }

      const videoDurationSec = Number.isFinite(video.duration) ? Math.max(video.duration, 0) : playbackVideoDurationSec;
      const offsetSec = videoTimeToPlaybackOffsetSec(
        video.currentTime,
        videoDurationSec,
        playbackDurationSec,
        playbackVideoSyncMap
      );
      const exactFrameIndex =
        playbackVideoSyncMap?.offsetsSec?.length
          ? videoTimeToPlaybackVideoFrameIndex(video.currentTime, playbackVideoSyncMap)
          : null;
      const paused = video.paused || video.ended;
      const rate = Math.max(0.1, video.playbackRate * computePlaybackVideoToBagScale(videoDurationSec, playbackDurationSec));
      const usingExactSyncMap = Boolean(playbackVideoSyncMap?.offsetsSec?.length);
      const now = performance.now();
      const latestEstimatedOffsetSec =
        latestStatus?.estimatedOffsetSec ?? latestStatus?.currentOffsetSec ?? Number.NaN;
      const driftSec = Number.isFinite(latestEstimatedOffsetSec)
        ? Math.abs(offsetSec - latestEstimatedOffsetSec)
        : Number.POSITIVE_INFINITY;
      const seekToleranceSec = usingExactSyncMap
        ? paused
          ? Math.max(0.9, playbackVideoToBagScale * 0.18)
          : Math.max(3.2, playbackVideoToBagScale * 0.4)
        : Math.max(0.6, playbackVideoToBagScale * 0.16);
      const shouldSendSeek =
        paused ||
        (driftSec > seekToleranceSec &&
          (force || now - playbackVideoLastSeekWallRef.current >= 900));

      if (usingExactSyncMap && !paused && exactFrameIndex !== null && !force && !shouldSendSeek) {
        if (
          playbackVideoLastFrameSyncRef.current === exactFrameIndex &&
          now - playbackVideoLastSyncWallRef.current < 160
        ) {
          return;
        }
      }
      const signature =
        usingExactSyncMap
          ? shouldSendSeek
            ? `${paused ? "paused" : "playing"}-seek:${exactFrameIndex ?? -1}:${offsetSec.toFixed(3)}:${rate.toFixed(3)}`
            : `playing-rate:${rate.toFixed(3)}`
          : `${paused ? "paused" : "playing"}:${rate.toFixed(3)}:${offsetSec.toFixed(3)}`;

      if (!force && playbackVideoLastSyncSignatureRef.current === signature) {
        return;
      }

      if (playbackVideoSyncInFlightRef.current) {
        playbackVideoPendingSyncRef.current = true;
        return;
      }

      playbackVideoSyncInFlightRef.current = true;
      playbackVideoLastSyncSignatureRef.current = signature;
      const ok = await sendPlaybackControl(
        usingExactSyncMap
          ? shouldSendSeek
            ? {
                paused,
                rate,
                offsetSec
              }
            : {
                paused: false,
                rate
              }
          : {
              rate,
              paused,
              offsetSec
            },
        {
          background: true
        }
      );
      playbackVideoSyncInFlightRef.current = false;

      if (!ok) {
        playbackVideoLastSyncSignatureRef.current = null;
      } else {
        playbackVideoLastSyncWallRef.current = now;
        if (shouldSendSeek) {
          playbackVideoLastSeekWallRef.current = now;
        }
        if (exactFrameIndex !== null) {
          playbackVideoLastFrameSyncRef.current = exactFrameIndex;
        }
      }

      if (playbackVideoPendingSyncRef.current) {
        playbackVideoPendingSyncRef.current = false;
        void syncPlaybackToMasterVideo(true);
      }
    },
    [
      consoleMode,
      playbackStatus,
      playbackVideoDurationSec,
      playbackVideoReady,
      playbackVideoSyncMap,
      playbackVideoToBagScale,
      sendPlaybackControl
    ]
  );

  const commitPlaybackVideoDraft = useCallback(async () => {
    if (playbackVideoDraftTimeSec === null) {
      return;
    }

    const video = playbackVideoRef.current;
    if (!video) {
      return;
    }

    const shouldResume = playbackVideoScrubbingRef.current
      ? playbackVideoScrubResumeRef.current
      : !video.paused && !video.ended;
    playbackVideoScrubbingRef.current = false;
    playbackVideoScrubResumeRef.current = false;
    pausePlaybackVideo();
    playbackOverlayDesiredRef.current = null;
    playbackOverlayLoadedKeyRef.current = null;
    playbackOverlayLastRequestedFrameRef.current = null;
    playbackOverlayLastRequestWallRef.current = 0;
    playbackOverlayFetchInFlightRef.current = false;
    playbackOverlayAbortRef.current?.abort();
    playbackOverlayAbortRef.current = null;
    setPlaybackOverlayTrajectory(null);
    setPlaybackOverlayScanPointCloud(null);
    await seekPlaybackVideo(playbackVideoDraftTimeSec);
    setPlaybackVideoDraftTimeSec(null);
    await syncPlaybackToMasterVideo(true);
    if (shouldResume) {
      const resumed = await requestPlaybackVideoPlay();
      if (resumed) {
        await syncPlaybackToMasterVideo(true);
      }
    }
  }, [
    pausePlaybackVideo,
    playbackVideoDraftTimeSec,
    requestPlaybackVideoPlay,
    seekPlaybackVideo,
    syncPlaybackToMasterVideo
  ]);

  const togglePlaybackVideo = useCallback(async () => {
    const video = playbackVideoRef.current;
    if (!video) {
      return;
    }

    if (video.paused || video.ended) {
      if (video.ended || video.currentTime >= Math.max(video.duration - 0.05, 0)) {
        playbackOverlayDesiredRef.current = null;
        playbackOverlayLoadedKeyRef.current = null;
        playbackOverlayLastRequestedFrameRef.current = null;
        playbackOverlayLastRequestWallRef.current = 0;
        playbackOverlayFetchInFlightRef.current = false;
        playbackOverlayAbortRef.current?.abort();
        playbackOverlayAbortRef.current = null;
        setPlaybackOverlayTrajectory(null);
        setPlaybackOverlayScanPointCloud(null);
        setPlaybackOverlayFrameIndex(null);
        playbackOverlayAccumulationRef.current = {
          voxels: new Map(),
          order: [],
          lastFrameIndex: null
        };
        setPlaybackOverlayAccumulatedPointCloud(null);
        setPlaybackResolvedMapPointCloud(null);
        await seekPlaybackVideo(0);
      }
      const started = await requestPlaybackVideoPlay();
      if (started) {
        await syncPlaybackToMasterVideo(true);
      }
      return;
    }

    pausePlaybackVideo();
    await syncPlaybackToMasterVideo(true);
  }, [pausePlaybackVideo, requestPlaybackVideoPlay, seekPlaybackVideo, syncPlaybackToMasterVideo]);

  const restartPlaybackVideo = useCallback(async () => {
    const video = playbackVideoRef.current;
    if (!video) {
      return;
    }

    const shouldResume = !video.paused && !video.ended;
    pausePlaybackVideo();
    playbackOverlayDesiredRef.current = null;
    playbackOverlayLoadedKeyRef.current = null;
    playbackOverlayLastRequestedFrameRef.current = null;
    playbackOverlayLastRequestWallRef.current = 0;
    playbackOverlayFetchInFlightRef.current = false;
    playbackOverlayAbortRef.current?.abort();
    playbackOverlayAbortRef.current = null;
    setPlaybackOverlayTrajectory(null);
    setPlaybackOverlayScanPointCloud(null);
    setPlaybackOverlayFrameIndex(null);
    playbackOverlayAccumulationRef.current = {
      voxels: new Map(),
      order: [],
      lastFrameIndex: null
    };
    setPlaybackOverlayAccumulatedPointCloud(null);
    setPlaybackResolvedMapPointCloud(null);
    await seekPlaybackVideo(0);
    setPlaybackVideoDraftTimeSec(null);
    await syncPlaybackToMasterVideo(true);
    if (shouldResume) {
      const resumed = await requestPlaybackVideoPlay();
      if (resumed) {
        await syncPlaybackToMasterVideo(true);
      }
    }
  }, [pausePlaybackVideo, requestPlaybackVideoPlay, seekPlaybackVideo, syncPlaybackToMasterVideo]);

  const setPlaybackVideoRateAndSync = useCallback(
    (rate: number) => {
      const video = playbackVideoRef.current;
      if (!video) {
        return;
      }

      video.playbackRate = rate;
      refreshPlaybackVideoState();
      void syncPlaybackToMasterVideo(true);
    },
    [refreshPlaybackVideoState, syncPlaybackToMasterVideo]
  );

  useEffect(() => {
    if (consoleMode === "playback") {
      return;
    }

    playbackVideoInitializedRef.current = false;
    playbackVideoPendingSyncRef.current = false;
    playbackVideoSyncInFlightRef.current = false;
    playbackVideoLastSyncSignatureRef.current = null;
    playbackVideoScrubbingRef.current = false;
    playbackVideoScrubResumeRef.current = false;
    playbackVideoLastFrameSyncRef.current = null;
    playbackVideoLastSyncWallRef.current = 0;
    playbackVideoLastSeekWallRef.current = 0;
    playbackVideoInitializingRef.current = false;
    playbackVideoCarryoverRef.current = null;
    playbackVideoCarryoverPendingRef.current = false;
    playbackOverlayLastRequestedFrameRef.current = null;
    playbackOverlayLastRequestWallRef.current = 0;
    setPlaybackVideoDraftTimeSec(null);

    const video = playbackVideoRef.current;
    if (video && !video.paused) {
      pausePlaybackVideo();
    }
    refreshPlaybackVideoState();
    setPlaybackResolvedMapPointCloud(null);
  }, [consoleMode, pausePlaybackVideo, refreshPlaybackVideoState]);

  useEffect(() => {
    if (
      consoleMode !== "playback" ||
      !playbackStatus ||
      !playbackVideoReady ||
      playbackVideoInitializedRef.current ||
      playbackVideoInitializingRef.current
    ) {
      return;
    }

    const video = playbackVideoRef.current;
    if (!video) {
      return;
    }
    let disposed = false;
    playbackVideoInitializingRef.current = true;
    const applyInitialVideoState = async () => {
      try {
        const targetVideoTimeSec = playbackOffsetToVideoTimeSec(
          playbackStatus.currentOffsetSec,
          playbackVideoDurationSec,
          playbackStatus.durationSec
        );
        await seekPlaybackVideo(targetVideoTimeSec);
        if (disposed) {
          return;
        }

        video.playbackRate = 1;

        if (playbackStatus.paused) {
          pausePlaybackVideo();
        } else {
          await requestPlaybackVideoPlay();
        }

        if (disposed) {
          return;
        }

        playbackVideoInitializedRef.current = true;
        playbackVideoLastSyncSignatureRef.current = null;
        refreshPlaybackVideoState();
        await syncPlaybackToMasterVideo(true);
      } finally {
        playbackVideoInitializingRef.current = false;
      }
    };

    void applyInitialVideoState();
    return () => {
      disposed = true;
      playbackVideoInitializingRef.current = false;
    };
  }, [
    consoleMode,
    playbackStatus?.currentOffsetSec,
    playbackStatus?.durationSec,
    playbackStatus?.paused,
    playbackVideoDurationSec,
    playbackVideoReady,
    pausePlaybackVideo,
    refreshPlaybackVideoState,
    requestPlaybackVideoPlay,
    seekPlaybackVideo,
    syncPlaybackToMasterVideo
  ]);

  useEffect(() => {
    if (consoleMode !== "playback" || playbackVideoPaused) {
      return;
    }

    const intervalId = window.setInterval(refreshPlaybackVideoState, 100);
    return () => {
      window.clearInterval(intervalId);
    };
  }, [consoleMode, playbackVideoPaused, refreshPlaybackVideoState]);

  useEffect(() => {
    if (
      consoleMode !== "playback" ||
      !playbackVideoReady ||
      playbackVideoPaused ||
      !playbackStatus
    ) {
      return;
    }

    if (playbackVideoSyncMap?.offsetsSec?.length) {
      const video = playbackVideoRef.current;
      if (video && "requestVideoFrameCallback" in video) {
        const frameVideo = video as HTMLVideoElement & {
          requestVideoFrameCallback(callback: () => void): number;
          cancelVideoFrameCallback?(handle: number): void;
        };
        let callbackId = 0;
        const tick = () => {
          void syncPlaybackToMasterVideo(false);
          callbackId = frameVideo.requestVideoFrameCallback(tick);
        };
        callbackId = frameVideo.requestVideoFrameCallback(tick);
        return () => {
          frameVideo.cancelVideoFrameCallback?.(callbackId);
        };
      }

      const intervalId = window.setInterval(() => {
        void syncPlaybackToMasterVideo(false);
      }, 85);
      return () => {
        window.clearInterval(intervalId);
      };
    }

    const intervalId = window.setInterval(() => {
      const video = playbackVideoRef.current;
      const latestStatus = playbackStatusRef.current;
      if (!video || !latestStatus) {
        return;
      }

      const expectedOffsetSec = videoTimeToPlaybackOffsetSec(
        video.currentTime,
        Number.isFinite(video.duration) ? Math.max(video.duration, 0) : playbackVideoDurationSec,
        latestStatus.durationSec,
        null
      );
      const driftSec = Math.abs(expectedOffsetSec - latestStatus.currentOffsetSec);
      const toleranceSec = Math.max(0.8, playbackVideoToBagScale * 0.18);
      if (driftSec > toleranceSec) {
        void syncPlaybackToMasterVideo(true);
      }
    }, 900);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [
    consoleMode,
    playbackStatus,
    playbackVideoDurationSec,
    playbackVideoPaused,
    playbackVideoReady,
    playbackVideoSyncMap,
    playbackVideoToBagScale,
    syncPlaybackToMasterVideo
  ]);

  useEffect(() => {
    if (!mainCanvasNode) {
      return;
    }

    let disposed = false;
    try {
      const viewer = new LiveGsViewer(mainCanvasNode);
      viewerRef.current = viewer;
      viewer.setMode(viewerMode);
      viewer.setPresentationMode(viewerPresentationMode);
      viewer.setGsControlMode(gsControlMode);
      viewer.setGaussianVariant(activeGaussianVariant);
      viewer.setVisibility(layers);
      viewer.setFollowRobot(effectiveFollowRobot);
      viewer.updateRobotPose(displayRobotPose);
      viewer.updateTrajectory(displayTrajectory);
      viewer.updatePlannedPath(plannedPath);
      viewer.updateLivePointCloud(rosState.livePointCloud);
      viewer.updatePlaybackMapPointCloud(playbackResolvedMapPointCloud);
      viewer.updatePlaybackScanPointCloud(
        consoleMode === "playback" ? playbackOverlayScan : rosState.playbackScanPointCloud
      );
      if (consoleMode === "hifi") {
        viewer.setCameraPose(HIFI_DEFAULT_CAMERA_POSE, HIFI_DEFAULT_TARGET);
        hifiInitializedRef.current = true;
      }
      viewer.setPlaybackCompareUsesWebGs(
        consoleMode === "playback" && !playbackShowcase && playbackCompareMode === "webgs"
      );
      if (effectiveManifest && consoleMode !== "hifi") {
        void viewer.loadManifest(effectiveManifest).then(
          () => {
            if (!disposed) {
              setSceneError(null);
            }
          },
          (error: unknown) => {
            if (!disposed) {
              setSceneError(error instanceof Error ? error.message : "Failed to load scene assets.");
            }
          }
        );
      }
      setSceneError(null);

      return () => {
        disposed = true;
        viewer.dispose();
        viewerRef.current = null;
      };
    } catch (error) {
      setSceneError(error instanceof Error ? error.message : "Failed to initialize 3D viewer.");
      viewerRef.current = null;
      return;
    }
  }, [mainCanvasNode]);

  useEffect(() => {
    viewerRef.current?.setMode(viewerMode);
  }, [viewerMode]);

  useEffect(() => {
    viewerRef.current?.setPresentationMode(viewerPresentationMode);
  }, [viewerPresentationMode]);

  useEffect(() => {
    if (consoleMode !== "hifi") {
      hifiInitializedRef.current = false;
      return;
    }

    if (hifiInitializedRef.current) {
      return;
    }

    viewerRef.current?.setCameraPose(HIFI_DEFAULT_CAMERA_POSE, HIFI_DEFAULT_TARGET);
    hifiInitializedRef.current = true;
  }, [consoleMode]);

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
    viewerRef.current?.setFollowRobot(effectiveFollowRobot);
  }, [effectiveFollowRobot]);

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
    viewerRef.current?.updatePlaybackMapPointCloud(playbackResolvedMapPointCloud);
  }, [playbackResolvedMapPointCloud]);

  useEffect(() => {
    viewerRef.current?.updatePlaybackScanPointCloud(
      consoleMode === "playback" ? playbackOverlayScan : rosState.playbackScanPointCloud
    );
  }, [consoleMode, playbackOverlayScan, rosState.playbackScanPointCloud]);

  useEffect(() => {
    if (!rosState.ros || consoleMode === "hifi" || consoleMode === "playback") {
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
  }, [consoleMode, rosState.ros]);

  useEffect(() => {
    setHifiStreamFailed(false);
  }, [consoleMode, hifiFrameUrl, hifiPreviewFrameUrl, hifiVideoStreamUrl]);

  useEffect(() => {
    if (consoleMode !== "hifi") {
      setHifiInteractive(false);
      return;
    }

    let disposed = false;
    const sync = () => {
      if (disposed) {
        return;
      }
      setHifiInteractive(Boolean(viewerRef.current?.isInteractionActive()));
    };

    sync();
    const intervalId = window.setInterval(sync, 50);
    return () => {
      disposed = true;
      window.clearInterval(intervalId);
    };
  }, [consoleMode]);

  useEffect(() => {
    if (!hifiActive) {
      return;
    }

    let disposed = false;

    const load = async () => {
      const candidates = buildControlUrlCandidates(hifiApiUrl, "/__hifi_bridge", 8876);
      let lastError: unknown = null;

      for (const candidate of candidates) {
        try {
          const response = await fetch(`${candidate}/status`, { cache: "no-store" });
          if (!response.ok) {
            throw new Error(`HiFi bridge returned ${response.status}`);
          }
          const status = (await response.json()) as HiFiStatus;
          if (!disposed) {
            if (candidate !== hifiApiUrl) {
              setHifiApiUrl(candidate);
            }
            setHifiStatus(status);
            setHifiError(null);
          }
          return;
        } catch (error) {
          lastError = error;
        }
      }

      if (!disposed) {
        setHifiError(lastError instanceof Error ? lastError.message : "HiFi bridge unavailable.");
      }
    };

    void load();
    const intervalId = window.setInterval(load, 1000);
    return () => {
      disposed = true;
      window.clearInterval(intervalId);
    };
  }, [hifiActive, hifiApiUrl]);

  useEffect(() => {
    if (!hifiActive) {
      return;
    }

    let disposed = false;
    const statusUrl = buildPlaybackVideoUrl(hifiStreamUrlBase, "/status");
    if (!statusUrl) {
      setHifiWindowStatus(null);
      return;
    }

    const load = async () => {
      try {
        const response = await fetch(statusUrl, { cache: "no-store" });
        if (!response.ok) {
          throw new Error(`HiFi window stream returned ${response.status}`);
        }
        const status = (await response.json()) as HiFiWindowStatus;
        if (!disposed) {
          setHifiWindowStatus(status);
        }
      } catch {
        if (!disposed) {
          setHifiWindowStatus(null);
        }
      }
    };

    void load();
    const intervalId = window.setInterval(load, 1000);
    return () => {
      disposed = true;
      window.clearInterval(intervalId);
    };
  }, [hifiActive, hifiStreamUrlBase]);

  useEffect(() => {
    if (!hifiActive || consoleMode === "playback" || (consoleMode === "hifi" && !hifiNativeRendererReady)) {
      hifiLastPublishedPoseRef.current = null;
      hifiLastPublishedAtRef.current = 0;
      return;
    }

    let disposed = false;
    let timeoutId = 0;
    const schedule = (delayMs: number) => {
      timeoutId = window.setTimeout(() => {
        void publishLoop();
      }, delayMs);
    };
    const publishLoop = async () => {
      if (disposed) {
        return;
      }

      const viewer = viewerRef.current;
      const interactive = viewer?.isInteractionActive() ?? false;
      const pose = viewer?.getCameraPose();
      const nextDelay = interactive ? 90 : 220;

      if (!pose) {
        schedule(nextDelay);
        return;
      }

      const previousPose = hifiLastPublishedPoseRef.current;
      const nowMs = performance.now();
      const poseChanged =
        !previousPose ||
        positionDistance3D(previousPose, pose) > (interactive ? 0.012 : 0.03) ||
        orientationDistance(previousPose, pose) > (interactive ? 0.004 : 0.01);
      const heartbeatMs = interactive ? 240 : 1200;
      const shouldSend = poseChanged || nowMs - hifiLastPublishedAtRef.current >= heartbeatMs;

      if (!shouldSend || hifiPoseRequestInFlightRef.current) {
        schedule(nextDelay);
        return;
      }

      hifiPoseRequestInFlightRef.current = true;
      try {
        const response = await fetch(`${hifiApiUrl}/camera_pose`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json"
          },
          body: JSON.stringify({
            position: pose.position,
            orientation: pose.orientation
          })
        });
        if (!response.ok) {
          throw new Error(`HiFi pose publish returned ${response.status}`);
        }
        hifiLastPublishedPoseRef.current = pose;
        hifiLastPublishedAtRef.current = performance.now();
        if (!disposed) {
          setHifiError(null);
        }
      } catch (error) {
        if (!disposed) {
          setHifiError(humanizeNetworkError(error, "HiFi camera pose"));
        }
      } finally {
        hifiPoseRequestInFlightRef.current = false;
      }

      schedule(nextDelay);
    };

    void publishLoop();
    return () => {
      disposed = true;
      window.clearTimeout(timeoutId);
      hifiPoseRequestInFlightRef.current = false;
    };
  }, [consoleMode, hifiActive, hifiApiUrl, hifiNativeRendererReady]);

  useEffect(() => {
    let disposed = false;

    const load = async () => {
      try {
        const response = await fetch(sceneUrl, { cache: "no-store" });
        if (!response.ok) {
          throw new Error(`Failed to load scene manifest: ${response.status}`);
        }
        const rawManifest = await response.text();
        if (disposed) {
          return;
        }
        if (sceneManifestSignatureRef.current === rawManifest) {
          return;
        }
        const nextManifest = JSON.parse(rawManifest) as SceneManifest;
        if (disposed) {
          return;
        }
        sceneManifestSignatureRef.current = rawManifest;
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

      if (consoleMode === "hifi") {
        if (!disposed) {
          setSceneError(null);
        }
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
  }, [consoleMode, effectiveManifest]);

  const applyConsoleMode = (nextMode: ConsoleMode) => {
    setSceneError(null);
    if (nextMode !== "playback") {
      setPlaybackBusy(false);
      setPlaybackError(null);
      setPlaybackVideoDraftTimeSec(null);
    }
    if (nextMode !== "hifi") {
      setHifiError(null);
      setHifiStreamFailed(false);
      setHifiInteractive(false);
    }
    if (nextMode !== "playback" && navPreviewActive) {
      setNavPreviewActive(false);
      setNavPreviewPose(null);
      setNavPreviewTrajectory([]);
      setNavPreviewMessage("Preview stopped.");
    }
    setConsoleMode(nextMode);
    setLayers((current) =>
      nextMode === "playback"
        ? {
            ...current,
            ...layerPresetForMode("playback"),
            ...layerPresetForPlaybackCloudMode(playbackCloudMode)
          }
        : {
            ...current,
            ...layerPresetForMode(nextMode)
          }
    );
  };

  const applyPlaybackCloudMode = (nextMode: PlaybackCloudMode) => {
    setPlaybackCloudMode(nextMode);
    setLayers((current) => ({
      ...current,
      ...layerPresetForPlaybackCloudMode(nextMode)
    }));
  };

  const applyPlaybackPresentation = (nextPresentation: PlaybackPresentation) => {
    if (!playbackShowcaseEnabled && nextPresentation === "showcase") {
      setPlaybackPresentation("workspace");
      return;
    }
    setPlaybackPresentation(nextPresentation);
    setSceneError(null);
    setPlaybackError(null);
    setHifiError(null);
    setHifiStreamFailed(false);
    if (nextPresentation === "showcase") {
      setGsControlMode("orbit");
      setFollowRobot(false);
      setLayers((current) => ({
        ...current,
        sdfMesh: false,
        gaussian: false,
        occupancyCloud: true,
        liveCloud: true,
        trajectory: true,
        robot: true
      }));
      setPlaybackCloudProfile("quality");
    }
  };

  useEffect(() => {
    if (!playbackShowcaseEnabled && playbackPresentation !== "workspace") {
      setPlaybackPresentation("workspace");
    }
  }, [playbackPresentation]);

  useEffect(() => {
    if (!playbackShowcase || (!layers.gaussian && !layers.sdfMesh)) {
      return;
    }
    setLayers((current) => ({
      ...current,
      gaussian: false,
      sdfMesh: false
    }));
  }, [layers.gaussian, layers.sdfMesh, playbackShowcase]);

  useEffect(() => {
    if (consoleMode !== "playback") {
      return;
    }

    setLayers((current) => {
      const needsCloudFallback = !current.occupancyCloud && !current.liveCloud;
      const next = {
        ...current,
        ...(needsCloudFallback ? layerPresetForPlaybackCloudMode(playbackCloudMode) : {})
      };
      const changed =
        next.occupancyCloud !== current.occupancyCloud ||
        next.liveCloud !== current.liveCloud;
      return changed ? next : current;
    });

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
    ? ["playback", "live", "gs", "hifi", "compare"]
    : ["playback", "live", "gs", "hifi"];
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

  const mapInPlaybackRail = consoleMode === "playback" && !playbackShowcase;

  const renderMapWorkspacePanel = (layout: "sidebar" | "playback-rail") => {
    const miniMapClassName = layout === "playback-rail" ? "mini-map mini-map-playback-rail" : "mini-map";
    const panelContent = (
      <>
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
        {consoleMode !== "playback" ? <p className="hint">{toolHint(mapTool)}</p> : null}
        {consoleMode !== "playback" ? (
          <>
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
                    setFollowRobot(true);
                    setNavPreviewPose(null);
                    setNavPreviewTrajectory([]);
                    setNavPreviewMessage("Preview initializing and following robot.");
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
          </>
        ) : null}
        {plannerMessage ? <p className="hint">{plannerMessage}</p> : null}
        {mapStatusMessage ? <p className="hint">{mapStatusMessage}</p> : null}
        {consoleMode !== "playback" && navPreviewMessage ? <p className="hint">{navPreviewMessage}</p> : null}
        <div className={miniMapClassName}>
          <TopDownMap
            manifest={manifest}
            ros={rosState.ros}
            streamProfile={rosState.streamProfile}
            robotPose={displayRobotPose}
            trajectory={displayTrajectory}
            mapPointCloud={playbackTopDownMapPointCloud}
            scanPointCloud={consoleMode === "playback" ? playbackOverlayScan : null}
            goals={goals}
            plannedPath={plannedPath}
            initialPose={initialPose}
            measurements={measurements}
            obstacles={obstacles}
            erasures={erasures}
            selectedTool={mapTool}
            followRobot={effectiveFollowRobot}
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
            ? "Goal / obstacle / erase edit the exported 2D map."
            : "Wheel zooms. Shift + drag pans."}
        </p>
      </>
    );

    if (layout === "playback-rail") {
      return (
        <section className="panel playback-map-shell">
          <div className="playback-map-shell-header">
            <div>
              <div className="overlay-title">Playback Map</div>
              <div className="playback-compare-title">2D map</div>
            </div>
            <label className="inline-check">
              <input
                type="checkbox"
                checked={followRobot}
                onChange={(event) => setFollowRobot(event.target.checked)}
              />
              <span>Follow robot</span>
            </label>
          </div>
          <div className="playback-map-shell-body">{panelContent}</div>
        </section>
      );
    }

    return (
      <InspectorSection
        title={consoleMode === "playback" ? "Playback Map" : "Navigation Workspace"}
        note={consoleMode === "playback" ? "2D map" : "2D nav"}
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
        {panelContent}
      </InspectorSection>
    );
  };

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
              {consoleMode === "hifi" ? (
                <>
                  <label className="endpoint">
                    <span>hifi-control</span>
                    <input value={hifiApiUrl} onChange={(event) => setHifiApiUrl(event.target.value)} />
                  </label>
                  <label className="endpoint">
                    <span>hifi-stream</span>
                    <input value={hifiStreamUrlBase} onChange={(event) => setHiFiStreamUrlBase(event.target.value)} />
                  </label>
                </>
              ) : null}
            </>
          ) : null}
        </div>
      </header>

      <main className={`workspace ${playbackShowcase ? "showcase-playback" : ""}`}>
        <div className="main-column">
        <section
          className={`viewer-panel ${consoleMode === "compare" ? "compare-layout" : ""} ${playbackCinemaLayout ? "playback-cinema-layout" : ""} ${playbackShowcase ? "showcase-viewer" : ""}`}
        >
          {playbackCinemaLayout ? (
            <>
              <div className="playback-cinema-map-stage">
                <div className="playback-cinema-map-overlay">
                  <div className="overlay-chip-row">
                    <span className="overlay-chip">{workspaceSemantic.badge}</span>
                    <span className="overlay-chip">2D map</span>
                    <div className="segmented-control compact-overlay-toggle">
                      <button
                        type="button"
                        className={`segmented-option ${playbackPresentation === "workspace" ? "active" : ""}`}
                        onClick={() => applyPlaybackPresentation("workspace")}
                      >
                        Workspace
                      </button>
                      {playbackShowcaseEnabled ? (
                        <button
                          type="button"
                          className={`segmented-option ${playbackPresentation === "showcase" ? "active" : ""}`}
                          onClick={() => applyPlaybackPresentation("showcase")}
                        >
                          Showcase
                        </button>
                      ) : null}
                    </div>
                  </div>
                </div>
                <div className="playback-cinema-map-layer">{renderMapWorkspacePanel("playback-rail")}</div>
              </div>
              <section className="playback-stream-rail">
                <div className="playback-stream-header">
                  <div>
                    <div className="overlay-title">Playback</div>
                    <div className="playback-stream-title">RGB · GS-SDF (stacked)</div>
                  </div>
                  <div className="segmented-control compare-mode-toggle">
                    <button
                      type="button"
                      className={`segmented-option ${playbackCompareMode === "video" ? "active" : ""}`}
                      onClick={() => setPlaybackCompareMode("video")}
                    >
                      Video
                    </button>
                    <button
                      type="button"
                      className={`segmented-option ${playbackCompareMode === "webgs" ? "active" : ""}`}
                      onClick={() => {
                        setSceneError(null);
                        setPlaybackError(null);
                        setHifiError(null);
                        setHifiStreamFailed(false);
                        setPlaybackCompareMode("webgs");
                        setGsControlMode("orbit");
                        setLayers((current) => ({
                          ...current,
                          gaussian: true
                        }));
                      }}
                    >
                      Spark
                    </button>
                  </div>
                </div>
                <div
                  className={`playback-stream-video-wrap ${
                    playbackCompareMode === "video" ? "" : "playback-stream-video-wrap-hidden"
                  }`}
                  aria-hidden={playbackCompareMode !== "video"}
                >
                  {renderPlaybackMasterVideo()}
                </div>
                {playbackCompareMode === "video" ? (
                  <>
                    <div className="playback-cinema-viewer-host" aria-hidden="true">
                      <canvas key={`viewer-canvas-video:${mediaSessionKey}`} ref={setMainCanvasRef} className="main-canvas" />
                    </div>
                  </>
                ) : (
                  <div className="playback-stream-stack">
                    <div className="playback-stream-tile">
                      <div className="feed-header">
                        <span>Playback RGB</span>
                        <strong>
                          {playbackStatus?.frameIndex !== undefined ? `f ${playbackStatus.frameIndex}` : "MJPEG"}
                        </strong>
                      </div>
                      <div className="playback-mjpeg-frame">
                        {playbackRgbMjpegUrl ? (
                          <img key={`playback-rgb:${mediaSessionKey}`} className="playback-mjpeg" src={playbackRgbMjpegUrl} alt="" />
                        ) : (
                          <div className="feed-empty">Configure playback-control URL</div>
                        )}
                      </div>
                    </div>
                    <div className="playback-stream-tile">
                      <div className="feed-header">
                        <span>GS-SDF</span>
                        <strong>
                          {playbackCompareMode === "webgs"
                            ? "Browser"
                            : hifiStatus?.ready || hifiWindowStatus?.ready
                              ? "MJPEG"
                              : "…"}
                        </strong>
                      </div>
                      <div
                        className={`playback-mjpeg-frame ${
                          playbackCompareMode === "live"
                            ? "playback-gs-hifi-stack"
                            : "playback-gs-webgs-frame"
                        }`}
                      >
                        {playbackCompareMode === "live" ? (
                          hifiVideoStreamUrl ? (
                            <img key={`playback-gs-live:${mediaSessionKey}`} className="playback-mjpeg" src={hifiVideoStreamUrl} alt="" />
                          ) : (
                            <div className="feed-empty">Configure HiFi bridge URL</div>
                          )
                        ) : null}
                        <div
                          className={
                            playbackCompareMode === "webgs"
                              ? "playback-webgs-viewer-host"
                              : "playback-cinema-viewer-host"
                          }
                        >
                          <canvas key={`viewer-canvas-compare:${mediaSessionKey}`} ref={setMainCanvasRef} className="main-canvas" />
                        </div>
                      </div>
                    </div>
                  </div>
                )}
                <p className="hint playback-stream-hint">{playbackCompareNote}</p>
              </section>
            </>
          ) : (
          <div className={`viewer-stage ${consoleMode === "hifi" && hifiNativeRendererReady ? "hifi-stage" : ""} ${playbackShowcase ? "showcase-stage" : ""}`}>
            {consoleMode === "playback" && playbackShowcase ? (
              <div className="playback-hidden-master-video" aria-hidden="true">
                {renderPlaybackMasterVideo("compare-video playback-master-video-hidden")}
              </div>
            ) : null}
            {consoleMode === "hifi" && hifiNativeRendererReady ? (
              <NativeRenderStage
                key={`native-stage:${mediaSessionKey}`}
                streamUrl={hifiVideoStreamUrl}
                frameUrl={hifiFrameUrl}
                previewFrameUrl={hifiPreviewFrameUrl}
                interactive={hifiInteractive}
                usingNativeWindow={hifiUsingNativeWindow}
                status={hifiStatus}
                windowStatus={hifiWindowStatus}
                error={hifiError}
                streamFailed={hifiStreamFailed}
                onStreamFailed={() => setHifiStreamFailed(true)}
              />
            ) : null}
            <canvas
              key={`viewer-canvas-main:${mediaSessionKey}`}
              ref={setMainCanvasRef}
              className={`main-canvas ${consoleMode === "hifi" && hifiNativeRendererReady ? "remote-control-canvas" : ""}`}
            />
            <div className="viewer-overlay top-left">
              <div className="overlay-card">
                <div className="overlay-title">Main 3D</div>
                <div className="overlay-value">{modeLabel}</div>
                <div className="overlay-hint">
                  {consoleMode === "playback"
                    ? playbackShowcase
                      ? "Showcase playback. Manual orbit with synced RGB inset, accumulated map, current scan, and trajectory accumulated from playback start."
                      : "Bag replay mode. FAST-LIVO2 sidecar publishes a world-frame global map and current scan. Left drag rotates, middle drag pans, wheel zooms."
                    : consoleMode === "hifi"
                    ? hifiNativeRendererReady
                      ? gsControlMode === "fps"
                        ? "Native GS-SDF render stream. Click viewport to lock mouse, then use WASD on the ground plane and Q/E for height."
                        : "Native GS-SDF render stream. Left drag rotates, middle drag pans, wheel zooms. Camera pose is forwarded to the backend renderer."
                      : "Native HiFi renderer offline. Falling back to browser GS so the scene stays visible."
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
            {sceneError ? <div className="viewer-error-overlay">{sceneError}</div> : null}
            <div className="viewer-overlay top-right">
              <div className="overlay-chip-row">
                <span className="overlay-chip">{workspaceSemantic.badge}</span>
                <span className="overlay-chip">{mainViewSource}</span>
                {consoleMode === "playback" ? (
                  <div className="segmented-control compact-overlay-toggle">
                    <button
                      type="button"
                      className={`segmented-option ${playbackPresentation === "workspace" ? "active" : ""}`}
                      onClick={() => applyPlaybackPresentation("workspace")}
                    >
                      Workspace
                    </button>
                    {playbackShowcaseEnabled ? (
                      <button
                        type="button"
                        className={`segmented-option ${playbackPresentation === "showcase" ? "active" : ""}`}
                        onClick={() => applyPlaybackPresentation("showcase")}
                      >
                        Showcase
                      </button>
                    ) : null}
                  </div>
                ) : null}
              </div>
            </div>
            {playbackShowcase ? (
              <div className="viewer-overlay bottom-left showcase-inset-wrap">
                <div className="showcase-inset-card">
                  <div className="showcase-inset-header">
                    <span>Playback RGB</span>
                    <strong>{playbackVisualPlaybackFrameIndex !== null ? `frame ${playbackVisualPlaybackFrameIndex}` : "waiting"}</strong>
                  </div>
                  <div className="showcase-inset-frame">
                    {playbackVideoElement && playbackVideoReady ? (
                      <PlaybackVideoMirror
                        key={`showcase-rgb:${mediaSessionKey}`}
                        sourceVideo={playbackVideoElement}
                        className="showcase-video-mirror"
                      />
                    ) : (
                      <div className="feed-empty">Waiting for playback RGB</div>
                    )}
                  </div>
                </div>
              </div>
            ) : null}
          </div>
          )}

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
          <section className={`timeline-dock ${playbackShowcase ? "showcase-timeline" : ""}`}>
            <div className="timeline-primary">
              <div className="timeline-status">
                <div className="overlay-title">Playback</div>
                <div className="timeline-title-row">
                  <div className="timeline-title">
                    {playbackVideoPaused ? "Paused clip" : "Running clip"}
                  </div>
                  <div className="timeline-readout">
                    <strong>{formatSeconds(effectivePlaybackVideoTimeSec)}</strong>
                    <span>/ {formatSeconds(playbackVideoDurationSec)}</span>
                  </div>
                </div>
              </div>

              <div className="timeline-footer">
                <button
                  type="button"
                  className="primary-button"
                  disabled={playbackTimelineDisabled}
                  onClick={() => void togglePlaybackVideo()}
                >
                  {playbackVideoPaused ? "Play" : "Pause"}
                </button>
                <button
                  type="button"
                  className="ghost-button"
                  disabled={playbackTimelineDisabled}
                  onClick={restartPlaybackVideo}
                >
                  Restart
                </button>
              </div>
            </div>

            <input
              className="range-input timeline-slider"
              type="range"
              min={0}
              max={Math.max(playbackVideoDurationSec, 1)}
              step={0.01}
              value={effectivePlaybackVideoTimeSec}
              disabled={playbackTimelineDisabled}
              onChange={(event) => updatePlaybackVideoDraft(Number(event.target.value))}
              onMouseUp={() => void commitPlaybackVideoDraft()}
              onTouchEnd={() => void commitPlaybackVideoDraft()}
              onKeyUp={() => void commitPlaybackVideoDraft()}
              onBlur={() => void commitPlaybackVideoDraft()}
            />

            <div className="timeline-control-grid">
              <div className="segmented-control wrap">
                {playbackVideoRates.map((rate) => (
                  <button
                    key={rate}
                    type="button"
                    className={`segmented-option ${Math.abs(playbackVideoRate - rate) < 0.01 ? "active" : ""}`}
                    disabled={playbackTimelineDisabled}
                    onClick={() => setPlaybackVideoRateAndSync(rate)}
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
                  {playbackVideoReady ? `video ${playbackVideoRate}x` : "video loading"}
                </span>
                <span className="overlay-chip">
                  {playbackVideoReady ? `bag ~${formatPlaybackRate(playbackVideoRate * playbackVideoToBagScale)}x` : "bag waiting"}
                </span>
                <span className="overlay-chip">
                  {playbackStatus ? `map ${formatSeconds(effectivePlaybackOffset)}` : "map waiting"}
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
                <span className="control-label">Presentation</span>
                <div className="segmented-control wrap">
                  <button
                    type="button"
                    className={`segmented-option ${playbackPresentation === "workspace" ? "active" : ""}`}
                    onClick={() => applyPlaybackPresentation("workspace")}
                  >
                    Workspace
                  </button>
                  {playbackShowcaseEnabled ? (
                    <button
                      type="button"
                      className={`segmented-option ${playbackPresentation === "showcase" ? "active" : ""}`}
                      onClick={() => applyPlaybackPresentation("showcase")}
                    >
                      Showcase
                    </button>
                  ) : null}
                </div>
              </div>

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

            {visiblePlaybackError ? <p className="error-line">{visiblePlaybackError}</p> : null}
          </section>
        ) : null}
        </div>

        <aside className="sidebar">
          {!mapInPlaybackRail ? renderMapWorkspacePanel("sidebar") : null}

          {consoleMode !== "compare" && consoleMode !== "playback" && consoleMode !== "hifi" && showAdvanced ? (
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
                playbackShowcase
                  ? (
                      [
                        ["occupancyCloud", occupancyCloudLabel],
                        ["liveCloud", liveCloudLabel],
                        ["trajectory", "Trajectory"],
                        ["robot", "Robot"]
                      ] as const
                    )
                  : (
                      [
                        ["gaussian", "Gaussian"],
                        ["sdfMesh", "SDF Mesh"],
                        ["occupancyCloud", occupancyCloudLabel],
                        ["liveCloud", liveCloudLabel],
                        ["trajectory", "Trajectory"],
                        ["robot", "Robot"]
                      ] as const
                    )
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
                    {layerAssetState(
                      key,
                      manifest,
                      hasLiveCloud,
                      consoleMode,
                      displayRobotPose,
                      displayTrajectory.length
                    )}
                  </span>
                </label>
              ))}
            </div>
            <div className="control-stack">
              {!playbackShowcase ? (
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
              ) : null}

              {!playbackShowcase ? (
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
              ) : null}

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
                  <strong>{displayTrajectory.length}</strong>
                </div>
                <div>
                  <span>Live cloud</span>
                  <strong>
                    {consoleMode === "playback" && rosState.playbackAccumulatedPointCloud
                      ? `${rosState.playbackAccumulatedPointCloud.renderedPointCount.toLocaleString()} map / ${playbackOverlayScan?.renderedPointCount.toLocaleString() ?? 0} scan`
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
                  <span>HiFi stream</span>
                  <strong>
                    {consoleMode === "hifi"
                      ? hifiError
                        ? "error"
                        : hifiStatus?.ready
                          ? `${hifiStatus.frameWidth}x${hifiStatus.frameHeight}`
                          : "waiting"
                      : "n/a"}
                  </strong>
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
    case "ultra":
      return 3;
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
    case "hifi":
      return {
        gaussian: true,
        sdfMesh: true,
        occupancyCloud: false,
        liveCloud: false,
        trajectory: false,
        robot: false
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
  streamProfile: "idle" | "neural" | "playback",
  hifiBrowserFallbackActive = false
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

  if (consoleMode === "hifi") {
    return {
      title: "High fidelity review",
      detail: hifiBrowserFallbackActive
        ? "Browser GS fallback while native GS-SDF renderer is offline"
        : "Native GS-SDF render stream with browser camera controls",
      badge: "HiFi"
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
  gsControlMode: GsControlMode,
  hifiBrowserFallbackActive = false
) {
  if (consoleMode === "playback") {
    return {
      title: "Progressive playback",
      detail: "Raw point cloud + RGB from bag replay"
    };
  }

  if (consoleMode === "hifi") {
    return {
      title: hifiBrowserFallbackActive
        ? `${titleCase(gaussianVariant)} fallback / ${gsControlMode === "fps" ? "FPS" : "Orbit"}`
        : `Native stream / ${gsControlMode === "fps" ? "FPS" : "Orbit"}`,
      detail: hifiBrowserFallbackActive
        ? "Browser Spark renderer fallback while native HiFi is unavailable"
        : "Original GS-SDF renderer frames streamed into the web console"
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
  consoleMode: ConsoleMode,
  robotPose: Pose3D | null,
  trajectoryCount: number
): string {
  switch (key) {
    case "sdfMesh":
      return manifest?.assets?.mesh ?? "n/a";
    case "occupancyCloud":
      return consoleMode === "playback" ? (hasLiveCloud ? "streaming" : "waiting") : manifest?.assets?.rawPointCloud ?? "n/a";
    case "liveCloud":
      return hasLiveCloud ? "streaming" : "waiting";
    case "trajectory":
      return trajectoryCount > 1 ? `${trajectoryCount} pts` : "waiting";
    case "robot":
      return robotPose ? "live" : "waiting";
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

function decodePlaybackOverlayScan(payload: PlaybackOverlayScanPayload | null): DecodedPointCloud | null {
  if (!payload || !payload.positions || payload.positions.length < 3) {
    return null;
  }
  const positions = new Float32Array(payload.positions);
  const colors =
    payload.colors && payload.colors.length === payload.positions.length
      ? Float32Array.from(payload.colors, (value) => value / 255)
      : undefined;
  const renderedPointCount =
    payload.renderedPointCount && payload.renderedPointCount > 0
      ? payload.renderedPointCount
      : Math.floor(positions.length / 3);
  return {
    frameId: payload.frameId ?? "world",
    stampMs: payload.stampMs ?? 0,
    sourcePointCount: payload.sourcePointCount ?? renderedPointCount,
    renderedPointCount,
    positions,
    colors
  };
}

function buildPlaybackTrajectoryFromPoseTrack(
  poses: Pose3D[],
  frameIndex: number,
  maxPoints: number
): Pose3D[] {
  if (poses.length < 1 || frameIndex < 0) {
    return [];
  }
  const clampedEnd = Math.min(frameIndex, poses.length - 1);
  const count = clampedEnd + 1;
  const limit = Math.max(1, maxPoints);
  if (count <= limit) {
    return poses.slice(0, clampedEnd + 1);
  }

  const stride = Math.max(1, Math.ceil(count / limit));
  const trajectory: Pose3D[] = [];
  for (let index = 0; index <= clampedEnd; index += stride) {
    trajectory.push(poses[index]);
  }
  const terminalPose = poses[clampedEnd];
  if (trajectory.length < 1 || trajectory[trajectory.length - 1]?.stampMs !== terminalPose.stampMs) {
    trajectory.push(terminalPose);
  }
  return trajectory;
}

function accumulatePlaybackOverlayMap(
  state: PlaybackLocalMapState,
  cloud: DecodedPointCloud,
  frameIndex: number,
  maxPoints: number,
  voxelSize: number
): DecodedPointCloud {
  if (cloud.renderedPointCount <= 0) {
    state.lastFrameIndex = frameIndex;
    return buildPlaybackLocalMapPointCloud(state, cloud.frameId || "world", cloud.stampMs ?? 0);
  }

  const positions = cloud.positions;
  const colors = cloud.colors;
  const clampedVoxelSize = Math.max(voxelSize, 1e-3);

  for (let index = 0; index < cloud.renderedPointCount; index += 1) {
    const offset = index * 3;
    const x = positions[offset];
    const y = positions[offset + 1];
    const z = positions[offset + 2];
    const key = [
      Math.round(x / clampedVoxelSize),
      Math.round(y / clampedVoxelSize),
      Math.round(z / clampedVoxelSize)
    ].join(":");
    const existing = state.voxels.get(key);
    if (existing) {
      existing.positionSamples += 1;
      const mix = 1 / existing.positionSamples;
      existing.x += (x - existing.x) * mix;
      existing.y += (y - existing.y) * mix;
      existing.z += (z - existing.z) * mix;
      if (colors) {
        const nextColor: [number, number, number] = [
          colors[offset],
          colors[offset + 1],
          colors[offset + 2]
        ];
        if (existing.color) {
          existing.colorSamples += 1;
          const colorMix = 1 / existing.colorSamples;
          existing.color[0] += (nextColor[0] - existing.color[0]) * colorMix;
          existing.color[1] += (nextColor[1] - existing.color[1]) * colorMix;
          existing.color[2] += (nextColor[2] - existing.color[2]) * colorMix;
        } else {
          existing.color = nextColor;
          existing.colorSamples = 1;
        }
      }
      continue;
    }

    const color: [number, number, number] | undefined = colors
      ? [colors[offset], colors[offset + 1], colors[offset + 2]]
      : undefined;
    state.voxels.set(key, {
      x,
      y,
      z,
      positionSamples: 1,
      color,
      colorSamples: color ? 1 : 0
    });
    state.order.push(key);
  }

  trimPlaybackLocalMap(state, maxPoints);
  state.lastFrameIndex = frameIndex;
  return buildPlaybackLocalMapPointCloud(state, cloud.frameId || "world", cloud.stampMs ?? 0);
}

function trimPlaybackLocalMap(state: PlaybackLocalMapState, maxPoints: number): void {
  if (maxPoints <= 0 || state.order.length <= maxPoints) {
    return;
  }
  const excess = state.order.length - maxPoints;
  for (let index = 0; index < excess; index += 1) {
    const key = state.order[index];
    state.voxels.delete(key);
  }
  state.order.splice(0, excess);
}

function buildPlaybackLocalMapPointCloud(
  state: PlaybackLocalMapState,
  frameId: string,
  stampMs: number
): DecodedPointCloud {
  const visibleKeys = state.order.filter((key) => state.voxels.has(key));
  const positions = new Float32Array(visibleKeys.length * 3);
  let colors: Float32Array | undefined;
  let hasColor = false;

  for (let index = 0; index < visibleKeys.length; index += 1) {
    const voxel = state.voxels.get(visibleKeys[index]);
    if (!voxel) {
      continue;
    }
    const offset = index * 3;
    positions[offset] = voxel.x;
    positions[offset + 1] = voxel.y;
    positions[offset + 2] = voxel.z;
    if (voxel.color) {
      colors ??= new Float32Array(visibleKeys.length * 3);
      colors[offset] = voxel.color[0];
      colors[offset + 1] = voxel.color[1];
      colors[offset + 2] = voxel.color[2];
      hasColor = true;
    }
  }

  return {
    frameId,
    stampMs,
    sourcePointCount: visibleKeys.length,
    renderedPointCount: visibleKeys.length,
    positions,
    colors: hasColor ? colors : undefined
  };
}

function pickPlaybackAccumulatedMapPointCloudForTarget(
  targetStampMs: number | null,
  resolvedAccumulatedMap: DecodedPointCloud | null | undefined,
  overlayAccumulatedMap: DecodedPointCloud | null | undefined,
  ...fallbackScans: Array<DecodedPointCloud | null | undefined>
): DecodedPointCloud | null {
  if (
    overlayAccumulatedMap &&
    (overlayAccumulatedMap.renderedPointCount ?? 0) > 0 &&
    targetStampMs !== null &&
    Number.isFinite(targetStampMs)
  ) {
    const overlayDelta = Math.abs((overlayAccumulatedMap.stampMs ?? 0) - targetStampMs);
    const resolvedDelta = resolvedAccumulatedMap
      ? Math.abs((resolvedAccumulatedMap.stampMs ?? 0) - targetStampMs)
      : Number.POSITIVE_INFINITY;
    if (overlayDelta <= resolvedDelta + 300) {
      return overlayAccumulatedMap;
    }
  }

  const cumulativeCandidates = [resolvedAccumulatedMap, overlayAccumulatedMap].filter(
    (candidate): candidate is DecodedPointCloud =>
      Boolean(candidate) && (candidate?.renderedPointCount ?? 0) > 0
  );
  if (cumulativeCandidates.length > 0) {
    if (targetStampMs === null || !Number.isFinite(targetStampMs)) {
      return cumulativeCandidates.reduce((best, candidate) =>
        candidate.renderedPointCount > best.renderedPointCount ? candidate : best
      );
    }

    let best = cumulativeCandidates[0];
    let bestDelta = Math.abs((best.stampMs ?? 0) - targetStampMs);

    for (const candidate of cumulativeCandidates.slice(1)) {
      const candidateDelta = Math.abs((candidate.stampMs ?? 0) - targetStampMs);
      if (candidateDelta + 120 < bestDelta) {
        best = candidate;
        bestDelta = candidateDelta;
        continue;
      }
      if (
        Math.abs(candidateDelta - bestDelta) <= 250 &&
        candidate.renderedPointCount > best.renderedPointCount
      ) {
        best = candidate;
        bestDelta = candidateDelta;
      }
    }

    return best;
  }
  const scanFallback = fallbackScans.find(
    (candidate) => Boolean(candidate) && (candidate?.renderedPointCount ?? 0) > 0
  );
  return scanFallback ?? null;
}

function computePlaybackVideoToBagScale(videoDurationSec: number, playbackDurationSec: number): number {
  if (videoDurationSec <= 0 || playbackDurationSec <= 0) {
    return 1;
  }
  return playbackDurationSec / videoDurationSec;
}

function videoTimeToPlaybackOffsetSec(
  videoTimeSec: number,
  videoDurationSec: number,
  playbackDurationSec: number,
  syncMap: PlaybackVideoSyncMap | null
): number {
  if (syncMap?.offsetsSec?.length) {
    const frameIndex = videoTimeToPlaybackVideoFrameIndex(videoTimeSec, syncMap);
    return syncMap.offsetsSec[frameIndex] ?? 0;
  }
  if (playbackDurationSec <= 0) {
    return Math.max(0, videoTimeSec);
  }
  if (videoDurationSec <= 0) {
    return clampNumber(videoTimeSec, 0, playbackDurationSec);
  }
  return clampNumber(videoTimeSec * computePlaybackVideoToBagScale(videoDurationSec, playbackDurationSec), 0, playbackDurationSec);
}

function playbackOffsetToVideoTimeSec(
  playbackOffsetSec: number,
  videoDurationSec: number,
  playbackDurationSec: number
): number {
  if (videoDurationSec <= 0) {
    return Math.max(0, playbackOffsetSec);
  }
  if (playbackDurationSec <= 0) {
    return clampNumber(playbackOffsetSec, 0, videoDurationSec);
  }
  return clampNumber(playbackOffsetSec * (videoDurationSec / playbackDurationSec), 0, videoDurationSec);
}

function videoTimeToPlaybackVideoFrameIndex(videoTimeSec: number, syncMap: PlaybackVideoSyncMap): number {
  const fps = Math.max(syncMap.fps, 1e-3);
  const lastFrameIndex = Math.max(syncMap.videoFrameCount - 1, 0);
  return Math.min(Math.max(Math.floor(videoTimeSec * fps + 1e-3), 0), lastFrameIndex);
}

function resolvePlaybackFrameIndexFromVideoTime(
  videoTimeSec: number,
  syncMap: PlaybackVideoSyncMap | null
): number | null {
  if (!syncMap?.frameIndices?.length) {
    return null;
  }
  const videoFrameIndex = videoTimeToPlaybackVideoFrameIndex(videoTimeSec, syncMap);
  const playbackFrameIndex = syncMap.frameIndices[videoFrameIndex];
  return Number.isFinite(playbackFrameIndex) ? Math.max(0, Math.round(playbackFrameIndex)) : null;
}

function clampNumber(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

function waitForPlaybackVideoSeek(video: HTMLVideoElement, targetTimeSec: number): Promise<void> {
  return new Promise((resolve) => {
    let settled = false;
    let timeoutId = 0;

    const finish = () => {
      if (settled) {
        return;
      }
      settled = true;
      window.clearTimeout(timeoutId);
      video.removeEventListener("seeked", finish);
      video.removeEventListener("loadeddata", maybeFinish);
      video.removeEventListener("timeupdate", maybeFinish);
      resolve();
    };

    const maybeFinish = () => {
      if (!video.seeking || Math.abs((video.currentTime || 0) - targetTimeSec) <= 0.05) {
        finish();
      }
    };

    timeoutId = window.setTimeout(finish, 300);
    video.addEventListener("seeked", finish);
    video.addEventListener("loadeddata", maybeFinish);
    video.addEventListener("timeupdate", maybeFinish);
    window.requestAnimationFrame(() => {
      window.requestAnimationFrame(maybeFinish);
    });
  });
}

function isBenignPlaybackVideoPlayError(error: unknown): boolean {
  if (!(error instanceof Error)) {
    return false;
  }
  const message = error.message.toLowerCase();
  return (
    message.includes("play() request was interrupted") ||
    message.includes("the fetching process for the media resource was aborted")
  );
}

function formatPlaybackRate(value: number): string {
  if (!Number.isFinite(value) || value <= 0) {
    return "0";
  }
  return value >= 10 ? value.toFixed(0) : value.toFixed(1).replace(/\.0$/, "");
}

function arePlaybackStatusesEquivalent(
  left: PlaybackStatus | null,
  right: PlaybackStatus | null
): boolean {
  if (left === right) {
    return true;
  }
  if (!left || !right) {
    return false;
  }

  return JSON.stringify({
    rate: left.rate,
    paused: left.paused,
    loop: left.loop,
    requestedOffsetSec: left.requestedOffsetSec,
    currentOffsetSec: left.currentOffsetSec,
    estimatedOffsetSec: left.estimatedOffsetSec,
    durationSec: left.durationSec,
    playbackAlive: left.playbackAlive,
    latestImageStampSec: left.latestImageStampSec,
    latestPoseStampSec: left.latestPoseStampSec,
    globalMapPoints: left.globalMapPoints,
    currentScanPoints: left.currentScanPoints,
    frameIndex: left.frameIndex,
    frameCount: left.frameCount,
    keyframeCount: left.keyframeCount,
    cacheDir: left.cacheDir,
    currentPose: left.currentPose,
    video: left.video
  }) === JSON.stringify({
    rate: right.rate,
    paused: right.paused,
    loop: right.loop,
    requestedOffsetSec: right.requestedOffsetSec,
    currentOffsetSec: right.currentOffsetSec,
    estimatedOffsetSec: right.estimatedOffsetSec,
    durationSec: right.durationSec,
    playbackAlive: right.playbackAlive,
    latestImageStampSec: right.latestImageStampSec,
    latestPoseStampSec: right.latestPoseStampSec,
    globalMapPoints: right.globalMapPoints,
    currentScanPoints: right.currentScanPoints,
    frameIndex: right.frameIndex,
    frameCount: right.frameCount,
    keyframeCount: right.keyframeCount,
    cacheDir: right.cacheDir,
    currentPose: right.currentPose,
    video: right.video
  });
}

function formatSeconds(value: number): string {
  const totalSeconds = Math.max(0, Math.floor(value));
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${minutes}:${seconds.toString().padStart(2, "0")}`;
}

function positionDistance3D(left: Pose3D, right: Pose3D): number {
  return Math.hypot(
    right.position.x - left.position.x,
    right.position.y - left.position.y,
    right.position.z - left.position.z
  );
}

function orientationDistance(left: Pose3D, right: Pose3D): number {
  const dot = Math.min(
    1,
    Math.max(
      -1,
      Math.abs(
        left.orientation.x * right.orientation.x +
          left.orientation.y * right.orientation.y +
          left.orientation.z * right.orientation.z +
          left.orientation.w * right.orientation.w
      )
    )
  );
  return 2 * Math.acos(dot);
}

function buildPlaybackVideoUrl(playbackApiUrl: string, path: string): string | null {
  try {
    const normalizedPath = path.startsWith("/") ? path : `/${path}`;
    if (playbackApiUrl.startsWith("/")) {
      return `${playbackApiUrl.replace(/\/+$/, "")}${normalizedPath}`;
    }

    const url = new URL(playbackApiUrl);
    const basePath = url.pathname === "/" ? "" : url.pathname.replace(/\/+$/, "");
    const isProxyBase = /\/__[^/]+$/.test(basePath);
    url.pathname = isProxyBase ? `${basePath}${normalizedPath}` : normalizedPath;
    url.search = "";
    return url.toString();
  } catch (error) {
    console.warn("Failed to build playback video URL.", error);
    return null;
  }
}

function resolvePlaybackMediaUrl(playbackApiUrl: string, candidate: string | null | undefined): string | null {
  if (!candidate) {
    return null;
  }
  if (/^(https?:|blob:|data:)/i.test(candidate)) {
    return candidate;
  }
  if (candidate.startsWith("/")) {
    return candidate;
  }
  return buildPlaybackVideoUrl(playbackApiUrl, candidate);
}

function humanizeNetworkError(error: unknown, context: string): string {
  if (error instanceof DOMException && error.name === "AbortError") {
    return `${titleCase(context)} request was interrupted.`;
  }
  if (error instanceof Error) {
    if (error.message === "Failed to fetch") {
      return `${titleCase(context)} is unreachable. Refresh the page or restart the related stack.`;
    }
    return error.message;
  }
  return `${titleCase(context)} is unavailable.`;
}

function buildControlUrlCandidates(preferredUrl: string, proxyPath: string, port: number): string[] {
  const candidates: string[] = [];
  const seen = new Set<string>();
  const add = (value: string | null) => {
    if (!value || seen.has(value)) {
      return;
    }
    seen.add(value);
    candidates.push(value);
  };

  add(preferredUrl);
  add(proxyPath);

  try {
    const current = new URL(window.location.href);
    add(`${current.origin}${proxyPath}`);
    add(`${current.protocol}//${current.hostname}:${port}`);
    if (current.hostname !== "localhost") {
      add(`${current.protocol}//localhost:${port}`);
    }
    if (current.hostname !== "127.0.0.1") {
      add(`${current.protocol}//127.0.0.1:${port}`);
    }
  } catch (error) {
    console.warn("Failed to build control URL candidates.", error);
  }

  return candidates;
}

function transformRobotPoseToPlaybackCameraPose(
  robotPose: Pose3D | null,
  camera: SceneManifest["camera"] | undefined
): Pose3D | null {
  if (!robotPose || !camera?.tBL || !camera?.tCL || camera.tBL.length !== 16 || camera.tCL.length !== 16) {
    return null;
  }

  const tWB = poseToMatrix(robotPose);
  const tBL = arrayToMatrix4(camera.tBL);
  const tCL = arrayToMatrix4(camera.tCL);
  const tLC = invertRigidTransform(tCL);
  const tBCOptical = multiplyMatrix4(tBL, tLC);
  const opticalToView = identityMatrix4();
  opticalToView[5] = -1;
  opticalToView[10] = -1;
  const tBCView = multiplyMatrix4(tBCOptical, opticalToView);
  const tWC = multiplyMatrix4(tWB, tBCView);
  return matrixToPose(tWC, robotPose.frameId);
}

function cameraTargetFromPose(pose: Pose3D, distance = 1): { x: number; y: number; z: number } {
  const { x, y, z, w } = pose.orientation;
  const forwardX = 2 * (x * z + w * y);
  const forwardY = 2 * (y * z - w * x);
  const forwardZ = 1 - 2 * (x * x + y * y);
  return {
    x: pose.position.x + forwardX * distance,
    y: pose.position.y + forwardY * distance,
    z: pose.position.z + forwardZ * distance
  };
}

function identityMatrix4(): number[] {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function arrayToMatrix4(values: number[]): number[] {
  return values.slice(0, 16);
}

function multiplyMatrix4(left: number[], right: number[]): number[] {
  const result = new Array<number>(16).fill(0);
  for (let row = 0; row < 4; row += 1) {
    for (let col = 0; col < 4; col += 1) {
      for (let k = 0; k < 4; k += 1) {
        result[row * 4 + col] += left[row * 4 + k] * right[k * 4 + col];
      }
    }
  }
  return result;
}

function invertRigidTransform(matrix: number[]): number[] {
  const rotation = [
    matrix[0], matrix[1], matrix[2],
    matrix[4], matrix[5], matrix[6],
    matrix[8], matrix[9], matrix[10]
  ];
  const translation = [matrix[3], matrix[7], matrix[11]];
  const rotationT = [
    rotation[0], rotation[3], rotation[6],
    rotation[1], rotation[4], rotation[7],
    rotation[2], rotation[5], rotation[8]
  ];
  const tx =
    -(rotationT[0] * translation[0] + rotationT[1] * translation[1] + rotationT[2] * translation[2]);
  const ty =
    -(rotationT[3] * translation[0] + rotationT[4] * translation[1] + rotationT[5] * translation[2]);
  const tz =
    -(rotationT[6] * translation[0] + rotationT[7] * translation[1] + rotationT[8] * translation[2]);
  return [
    rotationT[0], rotationT[1], rotationT[2], tx,
    rotationT[3], rotationT[4], rotationT[5], ty,
    rotationT[6], rotationT[7], rotationT[8], tz,
    0, 0, 0, 1
  ];
}

function poseToMatrix(pose: Pose3D): number[] {
  const { x, y, z, w } = pose.orientation;
  const xx = x * x;
  const yy = y * y;
  const zz = z * z;
  const xy = x * y;
  const xz = x * z;
  const yz = y * z;
  const wx = w * x;
  const wy = w * y;
  const wz = w * z;
  return [
    1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), pose.position.x,
    2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), pose.position.y,
    2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), pose.position.z,
    0, 0, 0, 1
  ];
}

function matrixToPose(matrix: number[], frameId: string): Pose3D {
  const m00 = matrix[0];
  const m01 = matrix[1];
  const m02 = matrix[2];
  const m10 = matrix[4];
  const m11 = matrix[5];
  const m12 = matrix[6];
  const m20 = matrix[8];
  const m21 = matrix[9];
  const m22 = matrix[10];
  const trace = m00 + m11 + m22;
  let x = 0;
  let y = 0;
  let z = 0;
  let w = 1;

  if (trace > 0) {
    const s = Math.sqrt(trace + 1.0) * 2;
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }

  return {
    frameId,
    position: { x: matrix[3], y: matrix[7], z: matrix[11] },
    orientation: { x, y, z, w }
  };
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

function PlaybackVideoMirror(props: { sourceVideo: HTMLVideoElement | null; className?: string }) {
  const { sourceVideo, className } = props;
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const contextRef = useRef<CanvasRenderingContext2D | null>(null);

  useEffect(() => {
    const video = sourceVideo;
    const canvas = canvasRef.current;
    if (!video || !canvas) {
      return;
    }

    let disposed = false;
    let callbackId = 0;
    let intervalId = 0;

    const getContext = () => {
      if (contextRef.current) {
        return contextRef.current;
      }
      const context = canvas.getContext("2d", { alpha: false, desynchronized: true });
      if (!context) {
        return null;
      }
      contextRef.current = context;
      return context;
    };

    const drawFrame = () => {
      if (disposed) {
        return;
      }
      const width = video.videoWidth || 0;
      const height = video.videoHeight || 0;
      if (width <= 0 || height <= 0 || video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA) {
        return;
      }
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }
      const context = getContext();
      if (!context) {
        return;
      }
      context.drawImage(video, 0, 0, width, height);
    };

    const stopLoop = () => {
      const frameVideo = video as HTMLVideoElement & {
        cancelVideoFrameCallback?(handle: number): void;
      };
      if (callbackId) {
        frameVideo.cancelVideoFrameCallback?.(callbackId);
        callbackId = 0;
      }
      if (intervalId) {
        window.clearInterval(intervalId);
        intervalId = 0;
      }
    };

    const startLoop = () => {
      stopLoop();
      drawFrame();
      if (video.paused || video.ended) {
        return;
      }
      if ("requestVideoFrameCallback" in video) {
        const frameVideo = video as HTMLVideoElement & {
          requestVideoFrameCallback(callback: () => void): number;
        };
        const tick = () => {
          drawFrame();
          callbackId = frameVideo.requestVideoFrameCallback(tick);
        };
        callbackId = frameVideo.requestVideoFrameCallback(tick);
        return;
      }
      intervalId = window.setInterval(drawFrame, 75);
    };

    const handlePassiveUpdate = () => {
      drawFrame();
      if (!video.paused && !video.ended) {
        startLoop();
      }
    };

    drawFrame();
    startLoop();
    video.addEventListener("loadedmetadata", handlePassiveUpdate);
    video.addEventListener("loadeddata", handlePassiveUpdate);
    video.addEventListener("seeked", handlePassiveUpdate);
    video.addEventListener("timeupdate", handlePassiveUpdate);
    video.addEventListener("play", startLoop);
    video.addEventListener("pause", handlePassiveUpdate);
    video.addEventListener("ratechange", handlePassiveUpdate);

    return () => {
      disposed = true;
      stopLoop();
      video.removeEventListener("loadedmetadata", handlePassiveUpdate);
      video.removeEventListener("loadeddata", handlePassiveUpdate);
      video.removeEventListener("seeked", handlePassiveUpdate);
      video.removeEventListener("timeupdate", handlePassiveUpdate);
      video.removeEventListener("play", startLoop);
      video.removeEventListener("pause", handlePassiveUpdate);
      video.removeEventListener("ratechange", handlePassiveUpdate);
    };
  }, [sourceVideo]);

  return <canvas ref={canvasRef} className={className} aria-label="Playback RGB inset" />;
}

function NativeRenderStage(props: {
  streamUrl: string | null;
  frameUrl: string | null;
  previewFrameUrl: string | null;
  interactive: boolean;
  usingNativeWindow: boolean;
  status: HiFiStatus | null;
  windowStatus: HiFiWindowStatus | null;
  error: string | null;
  streamFailed: boolean;
  onStreamFailed(): void;
}) {
  const {
    streamUrl,
    frameUrl,
    previewFrameUrl,
    interactive,
    usingNativeWindow,
    status,
    windowStatus,
    error,
    streamFailed,
    onStreamFailed
  } = props;
  const [displayUrl, setDisplayUrl] = useState<string | null>(null);
  const [frameError, setFrameError] = useState<string | null>(null);
  const [streamError, setStreamError] = useState<string | null>(null);
  const [streamLoaded, setStreamLoaded] = useState(false);
  const [streamTimedOut, setStreamTimedOut] = useState(false);
  const objectUrlRef = useRef<string | null>(null);
  const useStream = Boolean(streamUrl) && usingNativeWindow && !streamFailed && !streamError;
  const ready = (useStream || Boolean(frameUrl)) && !streamFailed;
  const statusLabel = error
    ? `HiFi bridge error: ${error}`
    : streamError
      ? `HiFi stream error: ${streamError}`
    : windowStatus?.ready
      ? `Native window stream ${windowStatus.width}x${windowStatus.height}`
      : status?.ready
      ? `Native renderer stream${status.frameWidth > 0 ? ` ${status.frameWidth}x${status.frameHeight}` : ""}${
          interactive && status.previewFrameWidth
            ? ` / preview ${status.previewFrameWidth}x${status.previewFrameHeight ?? 0}`
            : ""
        }`
      : windowStatus?.lastError
        ? `${windowStatus.lastError}. Falling back to renderer stream.`
      : "Waiting for native GS-SDF render stream";
  const activeFrameUrl = interactive && previewFrameUrl ? previewFrameUrl : frameUrl;

  useEffect(() => {
    setFrameError(null);
  }, [frameUrl, previewFrameUrl, interactive]);

  useEffect(() => {
    setStreamError(null);
    setStreamLoaded(false);
    setStreamTimedOut(false);
  }, [streamUrl, usingNativeWindow]);

  useEffect(() => {
    if (!useStream || streamLoaded) {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      setStreamTimedOut(true);
    }, 1200);
    return () => window.clearTimeout(timeoutId);
  }, [streamLoaded, useStream]);

  useEffect(() => {
    if (useStream || !activeFrameUrl || streamFailed) {
      if (objectUrlRef.current) {
        URL.revokeObjectURL(objectUrlRef.current);
        objectUrlRef.current = null;
      }
      setDisplayUrl(null);
      return;
    }

    let disposed = false;
    let timeoutId = 0;
    let inFlight = false;
    let controller: AbortController | null = null;

    const schedule = (delayMs: number) => {
      timeoutId = window.setTimeout(() => {
        void fetchLatestFrame();
      }, delayMs);
    };

    const fetchLatestFrame = async () => {
      if (disposed || inFlight) {
        return;
      }

      inFlight = true;
      controller = new AbortController();
      try {
        const separator = activeFrameUrl.includes("?") ? "&" : "?";
        const response = await fetch(`${activeFrameUrl}${separator}ts=${Date.now()}`, {
          cache: "no-store",
          signal: controller.signal
        });
        if (!response.ok) {
          throw new Error(`HiFi frame returned ${response.status}`);
        }
        const frameBlob = await response.blob();
        if (disposed) {
          return;
        }
        const nextObjectUrl = URL.createObjectURL(frameBlob);
        if (objectUrlRef.current) {
          URL.revokeObjectURL(objectUrlRef.current);
        }
        objectUrlRef.current = nextObjectUrl;
        setDisplayUrl(nextObjectUrl);
        setFrameError(null);
      } catch (fetchError) {
        if (disposed) {
          return;
        }
        if (fetchError instanceof DOMException && fetchError.name === "AbortError") {
          return;
        }
        if (usingNativeWindow) {
          onStreamFailed();
        } else {
          setFrameError(fetchError instanceof Error ? fetchError.message : "Failed to fetch HiFi frame.");
        }
      } finally {
        inFlight = false;
        controller = null;
        if (!disposed) {
          schedule(interactive ? 90 : 180);
        }
      }
    };

    void fetchLatestFrame();
    return () => {
      disposed = true;
      window.clearTimeout(timeoutId);
      controller?.abort();
      if (objectUrlRef.current) {
        URL.revokeObjectURL(objectUrlRef.current);
        objectUrlRef.current = null;
      }
    };
  }, [activeFrameUrl, interactive, streamFailed, useStream, usingNativeWindow]);

  return (
    <div className="main-remote-surface">
      {useStream && !streamTimedOut ? (
        <img
          className="main-remote-stream"
          src={streamUrl ?? undefined}
          alt="Native GS-SDF render stream"
          onLoad={() => {
            setStreamLoaded(true);
            setStreamTimedOut(false);
            setFrameError(null);
          }}
          onError={() => {
            if (usingNativeWindow) {
              onStreamFailed();
              return;
            }
            setStreamError("MJPEG stream failed.");
          }}
        />
      ) : ready && displayUrl ? (
        <img
          className="main-remote-stream"
          src={displayUrl}
          alt="Native GS-SDF render stream"
          onLoad={() => setFrameError(null)}
        />
      ) : null}
      {!ready || (useStream && !streamTimedOut ? !streamLoaded : !displayUrl) ? (
        <div className="main-remote-placeholder">
          {frameError ? `HiFi frame error: ${frameError}` : statusLabel}
        </div>
      ) : null}
    </div>
  );
}
