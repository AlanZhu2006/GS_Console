import * as THREE from "three";
import * as Spark from "@sparkjsdev/spark";
import { MOUSE } from "three";
import { Line2 } from "three/examples/jsm/lines/Line2.js";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type {
  GaussianChunk,
  LayerVisibility,
  Pose3D,
  SceneManifest
} from "../gs/gsSdfSceneAdapter";
import type { PlanPoint2D } from "../map/localPlanner";
import { loadPlyGeometry } from "../ply/loadPly";
import type { DecodedPointCloud } from "../ros/pointCloud2";

export type ViewerMode = "live" | "playback" | "gs";
export type GsControlMode = "orbit" | "fps";

interface KeyboardState {
  forward: boolean;
  backward: boolean;
  left: boolean;
  right: boolean;
  up: boolean;
  down: boolean;
  boost: boolean;
}

export class LiveGsViewer {
  private static readonly MAX_FPS_PITCH = THREE.MathUtils.degToRad(85);
  private static readonly FPS_MOUSE_SENSITIVITY = 0.0024;
  private static readonly INTERACTION_LOD_MS = 140;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(65, 1, 0.05, 1000);
  private readonly clock = new THREE.Clock();
  private readonly pointSpriteTexture = createPointSpriteTexture();
  private readonly orbitControls: OrbitControls;
  private readonly gaussianLayer = new THREE.Group();
  private readonly meshLayer = new THREE.Group();
  private readonly pointCloudLayer = new THREE.Group();
  private readonly staticPointCloudLayer = new THREE.Group();
  private readonly livePointCloudLayer = new THREE.Group();
  private readonly playbackMapPointCloudLayer = new THREE.Group();
  private readonly playbackScanPointCloudLayer = new THREE.Group();
  private readonly robotLayer = new THREE.Group();
  private readonly trajectoryLayer = new THREE.Group();
  private readonly trajectoryPoseMarkerLayer = new THREE.Group();
  private readonly plannedPathLayer = new THREE.Group();
  private readonly keyboard: KeyboardState = {
    forward: false,
    backward: false,
    left: false,
    right: false,
    up: false,
    down: false,
    boost: false
  };
  private animationHandle = 0;
  private manifest: SceneManifest | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private mode: ViewerMode = "live";
  private gsControlMode: GsControlMode = "orbit";
  private gaussianVariant = "balanced";
  private visibility: LayerVisibility = {
    gaussian: true,
    sdfMesh: true,
    occupancyCloud: false,
    liveCloud: false,
    trajectory: true,
    robot: true
  };
  private gaussianSourceKey: string | null = null;
  private gaussianLoadingKey: string | null = null;
  private gaussianLoadTask: Promise<void> | null = null;
  private gaussianLoadGeneration = 0;
  private meshUrl: string | null = null;
  private rawPointCloudUrl: string | null = null;
  private meshBounds: THREE.Box3 | null = null;
  private rawPointCloudBounds: THREE.Box3 | null = null;
  private gaussianBounds: THREE.Box3 | null = null;
  private robotMarker: THREE.Object3D;
  private trajectoryLine: Line2 | null = null;
  private plannedPathLine: THREE.Line | null = null;
  private livePointCloudObject: THREE.Points | null = null;
  private playbackMapPointCloudObject: THREE.Points | null = null;
  private playbackScanPointCloudObject: THREE.Points | null = null;
  private latestLivePointCloud: DecodedPointCloud | null = null;
  private latestPlaybackMapPointCloud: DecodedPointCloud | null = null;
  private latestPlaybackScanPointCloud: DecodedPointCloud | null = null;
  private livePointCloudRevealCount = 0;
  private livePointCloudTargetCount = 0;
  private livePointCloudBounds: THREE.Box3 | null = null;
  private playbackMapPointCloudBounds: THREE.Box3 | null = null;
  private followRobot = false;
  private cameraInteractionUntil = 0;
  private interactionRenderActive = false;
  private readonly followCameraOffset = new THREE.Vector3(-6, -6, 4);
  private readonly prefetchedGaussianUrls = new Set<string>();
  private gaussianPrefetchTask: Promise<void> | null = null;
  private fpsYaw = 0;
  private fpsPitch = 0;
  private pointerLocked = false;
  private readonly keyDownHandler = (event: KeyboardEvent) => this.handleKeyChange(true, event);
  private readonly keyUpHandler = (event: KeyboardEvent) => this.handleKeyChange(false, event);
  private readonly orbitStartHandler = () => this.bumpInteractionWindow(220);
  private readonly orbitChangeHandler = () => this.bumpInteractionWindow();
  private readonly orbitEndHandler = () => this.bumpInteractionWindow(90);
  private readonly pointerLockChangeHandler = () => {
    this.pointerLocked = document.pointerLockElement === this.canvas;
  };
  private readonly mouseMoveHandler = (event: MouseEvent) => this.handlePointerLook(event);

  constructor(private readonly canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: false,
      alpha: false
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    this.scene.background = new THREE.Color("#111318");
    this.scene.fog = new THREE.Fog("#111318", 30, 140);

    this.camera.position.set(-8, -10, 7);
    this.camera.up.set(0, 0, 1);

    this.orbitControls = new OrbitControls(this.camera, canvas);
    this.orbitControls.enableDamping = true;
    this.orbitControls.screenSpacePanning = true;
    this.orbitControls.mouseButtons = {
      LEFT: MOUSE.ROTATE,
      MIDDLE: MOUSE.PAN,
      RIGHT: MOUSE.PAN
    };
    this.orbitControls.target.set(0, 0, 1.2);
    this.orbitControls.update();
    this.orbitControls.addEventListener("start", this.orbitStartHandler);
    this.orbitControls.addEventListener("change", this.orbitChangeHandler);
    this.orbitControls.addEventListener("end", this.orbitEndHandler);

    this.canvas.tabIndex = 0;
    this.canvas.addEventListener("click", this.handleCanvasClick);
    window.addEventListener("keydown", this.keyDownHandler);
    window.addEventListener("keyup", this.keyUpHandler);
    document.addEventListener("pointerlockchange", this.pointerLockChangeHandler);
    document.addEventListener("mousemove", this.mouseMoveHandler);

    this.scene.add(new THREE.HemisphereLight("#f3f2d7", "#1a1d22", 1.8));

    const directional = new THREE.DirectionalLight("#fff5d0", 1.5);
    directional.position.set(8, -4, 16);
    this.scene.add(directional);

    const grid = new THREE.GridHelper(120, 120, "#7a91bd", "#314158");
    grid.rotateX(Math.PI / 2);
    const gridMaterial = grid.material as THREE.Material;
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.6;
    this.scene.add(grid);

    const axes = new THREE.AxesHelper(3);
    axes.position.set(0, 0, 0.02);
    this.scene.add(axes);

    this.robotMarker = createRobotMarker();
    this.robotLayer.add(this.robotMarker);

    this.pointCloudLayer.add(this.staticPointCloudLayer);
    this.pointCloudLayer.add(this.livePointCloudLayer);
    this.pointCloudLayer.add(this.playbackMapPointCloudLayer);
    this.pointCloudLayer.add(this.playbackScanPointCloudLayer);

    this.scene.add(this.gaussianLayer);
    this.scene.add(this.meshLayer);
    this.scene.add(this.pointCloudLayer);
    this.scene.add(this.trajectoryLayer);
    this.scene.add(this.trajectoryPoseMarkerLayer);
    this.scene.add(this.plannedPathLayer);
    this.scene.add(this.robotLayer);

    this.setMode("live");
    this.installResizeObserver();
    this.applyRenderProfile();
    this.renderLoop();
  }

  setMode(mode: ViewerMode): void {
    this.mode = mode;
    this.orbitControls.enabled = mode !== "gs" || this.gsControlMode === "orbit";

    if (mode === "gs") {
      if (this.gsControlMode === "fps") {
        this.snapCameraToRobot();
      } else {
        this.frameGsScene();
      }
    } else if (mode === "playback") {
      this.unlockPointerLook();
      this.framePlaybackScene();
    } else {
      this.unlockPointerLook();
      this.frameLiveScene();
    }

    this.applyRenderProfile();
    if (!this.shouldLoadGaussian()) {
      this.gaussianLoadGeneration += 1;
      this.gaussianLoadingKey = null;
    }
    void this.ensureDeferredAssets();
  }

  setGsControlMode(controlMode: GsControlMode): void {
    this.gsControlMode = controlMode;
    this.orbitControls.enabled = this.mode !== "gs" || controlMode === "orbit";

    if (controlMode === "fps") {
      this.snapCameraToRobot();
    } else {
      this.unlockPointerLook();
      if (this.mode === "gs") {
        this.frameGsScene();
      }
    }

    this.applyRenderProfile();
  }

  setGaussianVariant(variant: string | null): void {
    this.gaussianVariant = variant ?? "balanced";
    this.applyRenderProfile();
  }

  setVisibility(visibility: LayerVisibility): void {
    this.visibility = visibility;
    this.gaussianLayer.visible = visibility.gaussian;
    this.meshLayer.visible = visibility.sdfMesh;
    this.pointCloudLayer.visible = visibility.occupancyCloud || visibility.liveCloud;
    const playbackMode = this.mode === "playback";
    this.staticPointCloudLayer.visible = visibility.occupancyCloud && !playbackMode;
    this.livePointCloudLayer.visible = visibility.liveCloud && !playbackMode;
    this.playbackMapPointCloudLayer.visible = visibility.occupancyCloud && playbackMode;
    this.playbackScanPointCloudLayer.visible = visibility.liveCloud && playbackMode;
    if (visibility.liveCloud && !playbackMode && this.latestLivePointCloud) {
      this.renderLivePointCloud(this.latestLivePointCloud);
    }
    if (visibility.occupancyCloud && playbackMode && this.latestPlaybackMapPointCloud) {
      this.renderPlaybackMapPointCloud(this.latestPlaybackMapPointCloud);
    }
    if (visibility.liveCloud && playbackMode && this.latestPlaybackScanPointCloud) {
      this.renderPlaybackScanPointCloud(this.latestPlaybackScanPointCloud);
    }
    this.robotLayer.visible = visibility.robot;
    this.trajectoryLayer.visible = visibility.trajectory;
    this.trajectoryPoseMarkerLayer.visible = visibility.trajectory;
    this.plannedPathLayer.visible = visibility.trajectory;
    this.applyRenderProfile();
    if (!this.shouldLoadGaussian()) {
      this.gaussianLoadGeneration += 1;
      this.gaussianLoadingKey = null;
    }
    void this.ensureDeferredAssets();
  }

  setFollowRobot(followRobot: boolean): void {
    this.followRobot = followRobot;

    if (!followRobot) {
      return;
    }

    const currentOffset = this.camera.position.clone().sub(this.orbitControls.target);
    if (currentOffset.lengthSq() > 0.5) {
      this.followCameraOffset.copy(currentOffset);
    }
    this.syncFollowCamera(true);
  }

  async loadManifest(manifest: SceneManifest): Promise<void> {
    this.manifest = manifest;

    const work: Promise<void>[] = [];
    const nextGaussianKey = getGaussianSourceKey(manifest.gaussian);
    if (nextGaussianKey !== this.gaussianSourceKey && nextGaussianKey !== this.gaussianLoadingKey) {
      this.gaussianLoadGeneration += 1;
      this.clearGroup(this.gaussianLayer);
      this.gaussianSourceKey = null;
      this.gaussianLoadingKey = null;
    }

    if (manifest.mesh?.url && manifest.mesh.url !== this.meshUrl) {
      work.push(this.loadMesh(manifest.mesh.url));
    }

    if (manifest.rawPointCloud?.url && manifest.rawPointCloud.url !== this.rawPointCloudUrl) {
      work.push(this.loadRawPointCloud(manifest.rawPointCloud.url));
    }

    await Promise.all(work);
    void this.prefetchGaussianAssets(manifest);
    await this.ensureDeferredAssets();
    this.setVisibility(this.visibility);
  }

  updateRobotPose(pose: Pose3D | null): void {
    if (!pose) {
      return;
    }

    this.robotLayer.position.set(pose.position.x, pose.position.y, pose.position.z);
    this.robotMarker.quaternion.set(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );
    this.syncFollowCamera();
  }

  updateTrajectory(path: Pose3D[]): void {
    if (this.trajectoryLine) {
      this.trajectoryLayer.remove(this.trajectoryLine);
      this.trajectoryLine.geometry.dispose();
      this.trajectoryLine.material.dispose();
      this.trajectoryLine = null;
    }
    this.clearGroup(this.trajectoryPoseMarkerLayer);

    if (path.length < 2) {
      return;
    }

    const positions = new Array<number>(path.length * 3);
    const colors = new Array<number>(path.length * 3);

    for (let index = 0; index < path.length; index += 1) {
      const pose = path[index];
      const offset = index * 3;
      positions[offset] = pose.position.x;
      positions[offset + 1] = pose.position.y;
      positions[offset + 2] = pose.position.z + 0.07;

      const t = path.length <= 1 ? 0 : index / (path.length - 1);
      const color = new THREE.Color().setHSL((1 - t) * 0.74, 0.88, 0.58);
      colors[offset] = color.r;
      colors[offset + 1] = color.g;
      colors[offset + 2] = color.b;
    }

    const geometry = new LineGeometry();
    geometry.setPositions(positions);
    geometry.setColors(colors);

    const resolution = this.getViewportSize();
    const material = new LineMaterial({
      color: "#ffffff",
      vertexColors: true,
      linewidth: 6,
      transparent: true,
      opacity: 0.92,
      depthWrite: false,
      dashed: false
    });
    material.resolution.set(resolution.x, resolution.y);

    this.trajectoryLine = new Line2(geometry, material);
    this.trajectoryLine.computeLineDistances();
    this.trajectoryLayer.add(this.trajectoryLine);

    const markerStride = Math.max(1, Math.floor(path.length / 28));
    for (let index = 0; index < path.length; index += markerStride) {
      const pose = path[index];
      const t = path.length <= 1 ? 0 : index / (path.length - 1);
      const color = new THREE.Color().setHSL((1 - t) * 0.74, 0.88, 0.58);
      const marker = createTrajectoryPoseMarker(color);
      marker.position.set(pose.position.x, pose.position.y, pose.position.z + 0.06);
      marker.quaternion.set(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
      );
      this.trajectoryPoseMarkerLayer.add(marker);
    }
  }

  updatePlannedPath(path: PlanPoint2D[]): void {
    if (this.plannedPathLine) {
      this.plannedPathLayer.remove(this.plannedPathLine);
      this.plannedPathLine.geometry.dispose();
      (this.plannedPathLine.material as THREE.Material).dispose();
      this.plannedPathLine = null;
    }

    if (path.length < 2) {
      return;
    }

    const points = path.map((point) => new THREE.Vector3(point.x, point.y, 0.12));
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineDashedMaterial({
      color: "#ffd166",
      dashSize: 0.45,
      gapSize: 0.24
    });
    this.plannedPathLine = new THREE.Line(geometry, material);
    this.plannedPathLine.computeLineDistances();
    this.plannedPathLayer.add(this.plannedPathLine);
  }

  updateLivePointCloud(cloud: DecodedPointCloud | null): void {
    if (!cloud || cloud.renderedPointCount === 0) {
      this.latestLivePointCloud = null;
      this.livePointCloudRevealCount = 0;
      this.livePointCloudTargetCount = 0;
      if (this.livePointCloudObject) {
        this.livePointCloudLayer.remove(this.livePointCloudObject);
        this.livePointCloudObject.geometry.dispose();
        (this.livePointCloudObject.material as THREE.Material).dispose();
        this.livePointCloudObject = null;
      }
      return;
    }
    this.latestLivePointCloud = cloud;
    if (!this.visibility.liveCloud) {
      return;
    }
    this.renderLivePointCloud(cloud);
  }

  updatePlaybackMapPointCloud(cloud: DecodedPointCloud | null): void {
    this.latestPlaybackMapPointCloud = cloud;

    if (!cloud || cloud.renderedPointCount === 0) {
      this.playbackMapPointCloudBounds = null;
      this.disposePointCloudObject("playbackMap");
      return;
    }

    if (this.mode !== "playback" || !this.visibility.occupancyCloud) {
      return;
    }
    this.renderPlaybackMapPointCloud(cloud);
  }

  updatePlaybackScanPointCloud(cloud: DecodedPointCloud | null): void {
    this.latestPlaybackScanPointCloud = cloud;

    if (!cloud || cloud.renderedPointCount === 0) {
      this.disposePointCloudObject("playbackScan");
      return;
    }

    if (this.mode !== "playback" || !this.visibility.liveCloud) {
      return;
    }
    this.renderPlaybackScanPointCloud(cloud);
  }

  private renderLivePointCloud(cloud: DecodedPointCloud): void {
    const renderCloud = samplePointCloudForRender(
      cloud,
      this.interactionRenderActive ? 28_000 : 72_000
    );
    const nextBounds = computePointCloudBounds(renderCloud.positions);
    this.livePointCloudObject = this.renderPointCloudObject(
      this.livePointCloudObject,
      this.livePointCloudLayer,
      renderCloud,
      {
        defaultColor: "#78d8ff",
        sizeNear: 0.024,
        sizeFar: 0.014,
        opacity: 0.96,
        transparent: false,
        depthWrite: true,
        alphaTest: 0.42,
        crispSprite: true,
        bounds: nextBounds
      }
    );
    this.livePointCloudBounds = nextBounds;
    this.livePointCloudTargetCount = renderCloud.renderedPointCount;
    this.livePointCloudRevealCount = renderCloud.renderedPointCount;
  }

  private renderPlaybackMapPointCloud(cloud: DecodedPointCloud): void {
    const renderCloud = samplePointCloudForRender(cloud, this.interactionRenderActive ? 45_000 : 120_000);
    const hadBounds = Boolean(this.playbackMapPointCloudBounds);
    const nextBounds = computePointCloudBounds(renderCloud.positions);
    this.playbackMapPointCloudObject = this.renderPointCloudObject(
      this.playbackMapPointCloudObject,
      this.playbackMapPointCloudLayer,
      renderCloud,
      {
        defaultColor: "#d8ecff",
        sizeNear: 0.02,
        sizeFar: 0.012,
        opacity: 0.98,
        transparent: false,
        depthWrite: true,
        alphaTest: 0.42,
        crispSprite: true,
        bounds: nextBounds
      }
    );
    this.playbackMapPointCloudBounds = nextBounds;
    if (this.mode === "playback" && !this.followRobot && !hadBounds && this.playbackMapPointCloudBounds) {
      this.framePlaybackScene();
    }
  }

  private renderPlaybackScanPointCloud(cloud: DecodedPointCloud): void {
    const renderCloud = samplePointCloudForRender(cloud, this.interactionRenderActive ? 6_000 : 16_000);
    this.playbackScanPointCloudObject = this.renderPointCloudObject(
      this.playbackScanPointCloudObject,
      this.playbackScanPointCloudLayer,
      renderCloud,
      {
        defaultColor: "#ffd38c",
        sizeNear: 0.028,
        sizeFar: 0.018,
        opacity: 0.84,
        transparent: false,
        depthWrite: true,
        alphaTest: 0.48,
        crispSprite: true,
        bounds: this.playbackMapPointCloudBounds
      }
    );
  }

  private renderPointCloudObject(
    object: THREE.Points | null,
    layer: THREE.Group,
    cloud: DecodedPointCloud,
    options: {
      defaultColor: string;
      sizeNear: number;
      sizeFar: number;
      opacity: number;
      transparent?: boolean;
      depthWrite?: boolean;
      alphaTest?: number;
      crispSprite?: boolean;
      bounds?: THREE.Box3 | null;
    }
  ): THREE.Points {
    const useVertexColors = Boolean(cloud.colors && cloud.colors.length === cloud.positions.length);
    if (!object) {
      const geometry = new THREE.BufferGeometry();
      const material = new THREE.PointsMaterial({
        color: useVertexColors ? "#ffffff" : options.defaultColor,
        vertexColors: useVertexColors,
        size: options.sizeNear,
        sizeAttenuation: true,
        opacity: options.opacity,
        transparent: options.transparent ?? true,
        map: options.crispSprite ? this.pointSpriteTexture : null,
        alphaMap: options.crispSprite ? this.pointSpriteTexture : null,
        alphaTest: options.alphaTest ?? 0,
        fog: false,
        depthWrite: options.depthWrite ?? false
      });
      object = new THREE.Points(geometry, material);
      object.frustumCulled = false;
      layer.add(object);
    }

    const geometry = object.geometry as THREE.BufferGeometry;
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(cloud.positions, 3).setUsage(THREE.DynamicDrawUsage)
    );
    if (useVertexColors && cloud.colors) {
      geometry.setAttribute(
        "color",
        new THREE.BufferAttribute(cloud.colors, 3).setUsage(THREE.DynamicDrawUsage)
      );
    } else if (geometry.getAttribute("color")) {
      geometry.deleteAttribute("color");
    }
    geometry.setDrawRange(0, cloud.renderedPointCount);

    const material = object.material as THREE.PointsMaterial;
    material.vertexColors = useVertexColors;
    material.color.set(useVertexColors ? "#ffffff" : options.defaultColor);
    material.opacity = options.opacity;
    material.transparent = options.transparent ?? true;
    material.depthWrite = options.depthWrite ?? false;
    material.alphaTest = options.alphaTest ?? 0;
    material.map = options.crispSprite ? this.pointSpriteTexture : null;
    material.alphaMap = options.crispSprite ? this.pointSpriteTexture : null;
    material.size =
      this.resolvePointSize(
        cloud.renderedPointCount > 40_000
          ? options.sizeFar
          : cloud.renderedPointCount > 12_000
            ? (options.sizeNear + options.sizeFar) * 0.5
            : options.sizeNear,
        options.bounds ?? null
      );
    material.needsUpdate = true;
    return object;
  }

  getCameraPose(): Pose3D {
    return {
      frameId: "world",
      position: {
        x: this.camera.position.x,
        y: this.camera.position.y,
        z: this.camera.position.z
      },
      orientation: {
        x: this.camera.quaternion.x,
        y: this.camera.quaternion.y,
        z: this.camera.quaternion.z,
        w: this.camera.quaternion.w
      },
      stampMs: Date.now()
    };
  }

  dispose(): void {
    cancelAnimationFrame(this.animationHandle);
    this.gaussianLoadGeneration += 1;
    this.resizeObserver?.disconnect();
    this.canvas.removeEventListener("click", this.handleCanvasClick);
    window.removeEventListener("keydown", this.keyDownHandler);
    window.removeEventListener("keyup", this.keyUpHandler);
    document.removeEventListener("pointerlockchange", this.pointerLockChangeHandler);
    document.removeEventListener("mousemove", this.mouseMoveHandler);
    this.orbitControls.removeEventListener("start", this.orbitStartHandler);
    this.orbitControls.removeEventListener("change", this.orbitChangeHandler);
    this.orbitControls.removeEventListener("end", this.orbitEndHandler);
    this.orbitControls.dispose();
    this.pointSpriteTexture.dispose();
    this.renderer.dispose();
  }

  private async createGaussianObject(url: string): Promise<{
    object: THREE.Object3D;
    bounds: THREE.Box3 | null;
  }> {
    try {
      const SplatMeshCtor = (Spark as Record<string, unknown>).SplatMesh as
        | (new (options: { url: string }) => THREE.Object3D)
        | undefined;

      if (!SplatMeshCtor) {
        throw new Error("Spark SplatMesh export was not found.");
      }

      const splatMesh = new SplatMeshCtor({ url }) as THREE.Object3D & {
        initialized?: Promise<unknown>;
        getBoundingBox?: (centersOnly?: boolean) => THREE.Box3;
      };
      splatMesh.frustumCulled = false;
      if (splatMesh.initialized) {
        await splatMesh.initialized;
      }
      const splatBounds = splatMesh.getBoundingBox?.();
      return {
        object: splatMesh,
        bounds: splatBounds && isFiniteBox(splatBounds) ? splatBounds : null
      };
    } catch (error) {
      console.warn("Failed to initialize Spark gaussian layer.", error);
      const fallback = new THREE.Mesh(
        new THREE.BoxGeometry(1.2, 1.2, 1.2),
        new THREE.MeshStandardMaterial({
          color: "#c78f54",
          wireframe: true
        })
      );
      fallback.position.set(0, 0, 0.6);
      return {
        object: fallback,
        bounds: new THREE.Box3().setFromObject(fallback)
      };
    }
  }

  private async loadSingleGaussian(url: string, sourceKey: string, generation: number): Promise<void> {
    this.clearGroup(this.gaussianLayer);
    const { object, bounds } = await this.createGaussianObject(url);
    if (generation !== this.gaussianLoadGeneration) {
      disposeObject(object);
      return;
    }
    this.gaussianLayer.add(object);
    this.gaussianBounds = bounds;
    if (bounds && isFiniteBox(bounds)) {
      this.fitCameraToBox(bounds, 1.2);
    }
    this.gaussianSourceKey = sourceKey;
  }

  private async loadGaussianChunks(
    chunks: GaussianChunk[],
    sourceKey: string,
    generation: number
  ): Promise<void> {
    this.clearGroup(this.gaussianLayer);
    const combinedBounds = combineChunkBounds(chunks);
    const orderedChunks = orderGaussianChunks(chunks, this.camera.position, combinedBounds);

    if (combinedBounds && isFiniteBox(combinedBounds)) {
      this.gaussianBounds = combinedBounds;
      this.fitCameraToBox(combinedBounds, 1.05);
    }

    const initialBatch = orderedChunks.slice(0, 6);
    const deferredBatch = orderedChunks.slice(6);

    for (const chunk of initialBatch) {
      if (generation !== this.gaussianLoadGeneration) {
        return;
      }

      const { object, bounds } = await this.createGaussianObject(chunk.url);
      if (generation !== this.gaussianLoadGeneration) {
        disposeObject(object);
        return;
      }

      this.gaussianLayer.add(object);

      await new Promise((resolve) => window.setTimeout(resolve, 0));
    }

    window.setTimeout(async () => {
      for (const chunk of deferredBatch) {
        if (generation !== this.gaussianLoadGeneration) {
          return;
        }

        const { object } = await this.createGaussianObject(chunk.url);
        if (generation !== this.gaussianLoadGeneration) {
          disposeObject(object);
          return;
        }

        this.gaussianLayer.add(object);
        await new Promise((resolve) => window.setTimeout(resolve, 0));
      }

      if (generation === this.gaussianLoadGeneration) {
        this.gaussianBounds = combinedBounds;
        this.gaussianSourceKey = sourceKey;
      }
    }, 0);

    this.gaussianBounds = combinedBounds;
    this.gaussianSourceKey = sourceKey;
  }

  private async loadMesh(url: string): Promise<void> {
    this.clearGroup(this.meshLayer);
    const geometry = await loadPlyGeometry(url);
    geometry.computeBoundingBox();
    const position = geometry.getAttribute("position");
    const vertexCount = position?.count ?? 0;
    const useVertexColors = ensureGeometryVertexColors(geometry);
    const shouldComputeNormals =
      Boolean(geometry.index) && !geometry.getAttribute("normal") && vertexCount <= 600_000;

    if (shouldComputeNormals) {
      geometry.computeVertexNormals();
    }

    const material = geometry.index
      ? new THREE.MeshBasicMaterial({
          color: "#ffffff",
          vertexColors: useVertexColors,
          transparent: true,
          opacity: 0.96,
          side: THREE.DoubleSide,
          fog: false
        })
      : new THREE.PointsMaterial({
          color: useVertexColors ? "#ffffff" : "#d8dee6",
          vertexColors: useVertexColors,
          size: 0.034,
          map: this.pointSpriteTexture,
          alphaMap: this.pointSpriteTexture,
          alphaTest: 0.4,
          fog: false,
          depthWrite: true,
          transparent: false
        });

    const object = geometry.index
      ? new THREE.Mesh(geometry, material as THREE.MeshBasicMaterial)
      : new THREE.Points(geometry, material as THREE.PointsMaterial);

    this.meshLayer.add(object);
    const bounds = geometry.boundingBox ?? new THREE.Box3().setFromObject(object);
    this.meshBounds = bounds;
    if (isFiniteBox(bounds)) {
      this.frameLiveScene();
    }
    this.meshUrl = url;
  }

  private async loadRawPointCloud(url: string): Promise<void> {
    this.clearGroup(this.staticPointCloudLayer);
    const geometry = await loadPlyGeometry(url);
    geometry.computeBoundingBox();
    const useVertexColors = ensureGeometryVertexColors(geometry);
    const object = new THREE.Points(
      geometry,
      new THREE.PointsMaterial({
        size: 0.032,
        color: useVertexColors ? "#ffffff" : "#d7dee8",
        vertexColors: useVertexColors,
        map: this.pointSpriteTexture,
        alphaMap: this.pointSpriteTexture,
        alphaTest: 0.4,
        fog: false,
        depthWrite: true,
        transparent: false
      })
    );
    this.staticPointCloudLayer.add(object);
    const bounds = geometry.boundingBox ?? new THREE.Box3().setFromObject(object);
    this.rawPointCloudBounds = bounds;
    if (isFiniteBox(bounds)) {
      this.frameLiveScene();
    }
    this.rawPointCloudUrl = url;
  }

  private clearGroup(group: THREE.Group): void {
    for (const child of [...group.children]) {
      group.remove(child);
      if ("geometry" in child && child.geometry instanceof THREE.BufferGeometry) {
        child.geometry.dispose();
      }
      if ("material" in child) {
        const material = child.material;
        if (Array.isArray(material)) {
          material.forEach((item) => item.dispose());
        } else if (material instanceof THREE.Material) {
          material.dispose();
        }
      }
    }
  }

  private renderLoop = (): void => {
    const delta = this.clock.getDelta();
    this.animationHandle = requestAnimationFrame(this.renderLoop);
    this.syncInteractionRenderState();

    if (this.mode === "live" || this.gsControlMode === "orbit") {
      this.orbitControls.update();
      this.syncFollowCamera();
    } else {
      this.updateFps(delta);
      this.clampFpsOrientation();
    }

    this.updateLivePointCloudReveal(delta);

    this.renderer.render(this.scene, this.camera);
  };

  private updateLivePointCloudReveal(delta: number): void {
    if (!this.livePointCloudObject) {
      return;
    }

    if (this.livePointCloudRevealCount >= this.livePointCloudTargetCount) {
      return;
    }

    const remaining = this.livePointCloudTargetCount - this.livePointCloudRevealCount;
    const step = Math.max(
      Math.ceil(remaining * Math.min(delta * 14, 0.55)),
      Math.ceil(9000 * delta)
    );
    this.livePointCloudRevealCount = Math.min(
      this.livePointCloudRevealCount + step,
      this.livePointCloudTargetCount
    );
    (this.livePointCloudObject.geometry as THREE.BufferGeometry).setDrawRange(
      0,
      this.livePointCloudRevealCount
    );
  }

  private updateFps(delta: number): void {
    if (this.mode !== "gs" || this.gsControlMode !== "fps") {
      return;
    }

    const speed = this.keyboard.boost ? 12 : 4.5;
    const distance = speed * delta;

    if (!this.pointerLocked) {
      return;
    }

    const forward = new THREE.Vector3(Math.cos(this.fpsYaw), Math.sin(this.fpsYaw), 0);
    const strafe = new THREE.Vector3(-Math.sin(this.fpsYaw), Math.cos(this.fpsYaw), 0);

    if (this.keyboard.forward) {
      this.camera.position.addScaledVector(forward, distance);
    }
    if (this.keyboard.backward) {
      this.camera.position.addScaledVector(forward, -distance);
    }
    if (this.keyboard.left) {
      this.camera.position.addScaledVector(strafe, distance);
    }
    if (this.keyboard.right) {
      this.camera.position.addScaledVector(strafe, -distance);
    }
    if (this.keyboard.up) {
      this.camera.position.z += distance;
    }
    if (this.keyboard.down) {
      this.camera.position.z -= distance;
    }
  }

  private handleCanvasClick = (): void => {
    if (this.mode !== "gs" || this.gsControlMode !== "fps" || this.pointerLocked) {
      return;
    }
    void this.canvas.requestPointerLock();
  };

  private handleKeyChange(active: boolean, event: KeyboardEvent): void {
    switch (event.code) {
      case "KeyW":
        this.keyboard.forward = active;
        break;
      case "KeyS":
        this.keyboard.backward = active;
        break;
      case "KeyA":
        this.keyboard.left = active;
        break;
      case "KeyD":
        this.keyboard.right = active;
        break;
      case "KeyQ":
        this.keyboard.down = active;
        break;
      case "KeyE":
        this.keyboard.up = active;
        break;
      case "ShiftLeft":
      case "ShiftRight":
        this.keyboard.boost = active;
        break;
      default:
        break;
    }
  }

  private snapCameraToRobot(): void {
    const position = this.robotLayer.position;
    this.camera.position.set(position.x - 1.2, position.y - 1.2, position.z + 1.3);
    this.camera.up.set(0, 0, 1);

    const forward = this.orbitControls.target.clone().sub(this.camera.position);
    if (forward.lengthSq() < 1e-4) {
      forward.set(1, 0, 0);
    }
    forward.normalize();

    this.fpsYaw = Math.atan2(forward.y, forward.x);
    this.fpsPitch = THREE.MathUtils.clamp(
      Math.asin(THREE.MathUtils.clamp(forward.z, -1, 1)),
      -LiveGsViewer.MAX_FPS_PITCH,
      LiveGsViewer.MAX_FPS_PITCH
    );
    this.applyFpsOrientation();
  }

  private clampFpsOrientation(): void {
    this.fpsPitch = THREE.MathUtils.clamp(
      this.fpsPitch,
      -LiveGsViewer.MAX_FPS_PITCH,
      LiveGsViewer.MAX_FPS_PITCH
    );
    this.applyFpsOrientation();
  }

  private applyFpsOrientation(): void {
    const horizontal = Math.cos(this.fpsPitch);
    const direction = new THREE.Vector3(
      Math.cos(this.fpsYaw) * horizontal,
      Math.sin(this.fpsYaw) * horizontal,
      Math.sin(this.fpsPitch)
    );
    const lookTarget = this.camera.position.clone().add(direction);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(lookTarget);
  }

  private handlePointerLook(event: MouseEvent): void {
    if (!this.pointerLocked || this.mode !== "gs" || this.gsControlMode !== "fps") {
      return;
    }

    this.fpsYaw -= event.movementX * LiveGsViewer.FPS_MOUSE_SENSITIVITY;
    this.fpsPitch = THREE.MathUtils.clamp(
      this.fpsPitch - event.movementY * LiveGsViewer.FPS_MOUSE_SENSITIVITY,
      -LiveGsViewer.MAX_FPS_PITCH,
      LiveGsViewer.MAX_FPS_PITCH
    );
    this.applyFpsOrientation();
  }

  private unlockPointerLook(): void {
    if (document.pointerLockElement === this.canvas) {
      void document.exitPointerLock();
    }
    this.pointerLocked = false;
  }

  private frameLiveScene(): void {
    if (this.mode !== "live") {
      return;
    }

    if (this.followRobot) {
      this.syncFollowCamera(true);
      return;
    }

    const preferredBounds = this.getPreferredLiveBounds();
    if (preferredBounds) {
      this.fitCameraToBox(preferredBounds, 1.08);
      return;
    }

    this.camera.position.set(-8, -10, 7);
    this.camera.up.set(0, 0, 1);
    this.orbitControls.target.set(0, 0, 1.2);
    this.orbitControls.update();
  }

  private framePlaybackScene(): void {
    if (this.mode !== "playback") {
      return;
    }

    if (this.followRobot) {
      this.syncFollowCamera(true);
      return;
    }

    const preferredBounds = this.playbackMapPointCloudBounds ?? this.getPreferredLiveBounds();
    if (preferredBounds && isFiniteBox(preferredBounds)) {
      this.fitCameraToBox(preferredBounds, 1.1);
      return;
    }

    this.camera.position.set(-8, -10, 7);
    this.camera.up.set(0, 0, 1);
    this.orbitControls.target.set(0, 0, 1.2);
    this.orbitControls.update();
  }

  private frameGsScene(): void {
    if (this.mode !== "gs") {
      return;
    }

    if (this.followRobot && this.gsControlMode !== "fps") {
      this.syncFollowCamera(true);
      return;
    }

    const preferredBounds = this.gaussianBounds ?? this.meshBounds ?? this.rawPointCloudBounds;
    if (preferredBounds && isFiniteBox(preferredBounds)) {
      this.fitCameraToBox(preferredBounds, 1.04);
      return;
    }

    this.camera.position.set(-8, -10, 7);
    this.camera.up.set(0, 0, 1);
    this.orbitControls.target.set(0, 0, 1.2);
    this.orbitControls.update();
  }

  private getPreferredLiveBounds(): THREE.Box3 | null {
    const bounds = [this.livePointCloudBounds, this.meshBounds, this.rawPointCloudBounds, this.gaussianBounds].filter(
      (box): box is THREE.Box3 => Boolean(box && isFiniteBox(box))
    );

    if (!bounds.length) {
      return null;
    }

    const combined = bounds[0].clone();
    for (let index = 1; index < bounds.length; index += 1) {
      combined.union(bounds[index]);
    }
    return combined;
  }

  private fitCameraToBox(box: THREE.Box3, padding = 1.15): void {
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());

    const maxDimension = Math.max(size.x, size.y, size.z, 1);
    const distance =
      (maxDimension * padding) / (2 * Math.tan(THREE.MathUtils.degToRad(this.camera.fov * 0.5)));

    const direction = new THREE.Vector3(-1, -1, 0.6).normalize();
    const position = center.clone().add(direction.multiplyScalar(distance * 1.4));

    this.camera.position.copy(position);
    this.camera.near = Math.max(distance / 500, 0.05);
    this.camera.far = Math.max(distance * 25, 1000);
    this.camera.updateProjectionMatrix();

    this.orbitControls.target.copy(center);
    this.orbitControls.update();
  }

  private syncFollowCamera(force = false): void {
    if (!this.followRobot) {
      return;
    }
    if (this.mode === "gs" && this.gsControlMode === "fps") {
      return;
    }

    const target = this.robotLayer.position.clone();
    target.z += 0.9;
    const desiredPosition = target.clone().add(this.followCameraOffset);

    if (force) {
      this.camera.position.copy(desiredPosition);
      this.orbitControls.target.copy(target);
      this.orbitControls.update();
      return;
    }

    this.camera.position.lerp(desiredPosition, 0.14);
    this.orbitControls.target.lerp(target, 0.18);
  }

  private applyRenderProfile(): void {
    const gaussianActive = this.mode === "gs" && this.visibility.gaussian;
    const heavyLiveCloud =
      (
        (this.mode === "playback"
          ? (this.latestPlaybackMapPointCloud?.renderedPointCount ?? 0)
          : (this.latestLivePointCloud?.renderedPointCount ?? 0)
        ) > 45_000
      ) && (this.visibility.liveCloud || this.visibility.occupancyCloud);
    const maxPixelRatio = gaussianActive
      ? this.gsControlMode === "fps"
        ? 0.8
        : this.gaussianVariant === "quality"
          ? 1.0
          : this.gaussianVariant === "fast"
            ? 0.82
            : 0.9
      : this.interactionRenderActive
        ? 0.68
      : heavyLiveCloud
        ? 0.8
        : 1.0;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, maxPixelRatio));

    const parent = this.canvas.parentElement;
    if (!parent) {
      return;
    }
    const width = Math.max(parent.clientWidth, 1);
    const height = Math.max(parent.clientHeight, 1);
    this.renderer.setSize(width, height, false);
    this.updateLineMaterialResolution(width, height);
  }

  private installResizeObserver(): void {
    const resize = () => {
      const parent = this.canvas.parentElement;
      if (!parent) {
        return;
      }
      const width = Math.max(parent.clientWidth, 1);
      const height = Math.max(parent.clientHeight, 1);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height, false);
      this.updateLineMaterialResolution(width, height);
    };

    resize();
    this.resizeObserver = new ResizeObserver(resize);
    const parent = this.canvas.parentElement;
    if (parent) {
      this.resizeObserver.observe(parent);
    }
  }

  private getViewportSize(): THREE.Vector2 {
    const parent = this.canvas.parentElement;
    return new THREE.Vector2(
      Math.max(parent?.clientWidth ?? this.canvas.clientWidth, 1),
      Math.max(parent?.clientHeight ?? this.canvas.clientHeight, 1)
    );
  }

  private updateLineMaterialResolution(width: number, height: number): void {
    if (this.trajectoryLine) {
      this.trajectoryLine.material.resolution.set(width, height);
    }
  }

  private bumpInteractionWindow(durationMs = LiveGsViewer.INTERACTION_LOD_MS): void {
    this.cameraInteractionUntil = Math.max(this.cameraInteractionUntil, performance.now() + durationMs);
  }

  private syncInteractionRenderState(): void {
    const nextState = performance.now() < this.cameraInteractionUntil;
    if (nextState === this.interactionRenderActive) {
      return;
    }

    this.interactionRenderActive = nextState;
    this.refreshDynamicPointClouds();
    this.applyRenderProfile();
  }

  private refreshDynamicPointClouds(): void {
    if (this.mode === "playback") {
      if (this.latestPlaybackMapPointCloud && this.visibility.occupancyCloud) {
        this.renderPlaybackMapPointCloud(this.latestPlaybackMapPointCloud);
      }
      if (this.latestPlaybackScanPointCloud && this.visibility.liveCloud) {
        this.renderPlaybackScanPointCloud(this.latestPlaybackScanPointCloud);
      }
      return;
    }

    if (this.latestLivePointCloud && this.visibility.liveCloud) {
      this.renderLivePointCloud(this.latestLivePointCloud);
    }
  }

  private resolvePointSize(baseSize: number, bounds: THREE.Box3 | null): number {
    const interactionScale = this.interactionRenderActive ? 0.78 : 1;
    if (!bounds || !isFiniteBox(bounds)) {
      return baseSize * interactionScale;
    }

    const center = bounds.getCenter(new THREE.Vector3());
    const diagonal = Math.max(bounds.getSize(new THREE.Vector3()).length(), 1);
    const distance = this.camera.position.distanceTo(center);
    const distanceScale = THREE.MathUtils.clamp(0.46 + (distance / diagonal) * 0.34, 0.5, 1.18);
    return baseSize * distanceScale * interactionScale;
  }

  private disposePointCloudObject(kind: "live" | "playbackMap" | "playbackScan"): void {
    if (kind === "live" && this.livePointCloudObject) {
      this.livePointCloudLayer.remove(this.livePointCloudObject);
      this.livePointCloudObject.geometry.dispose();
      (this.livePointCloudObject.material as THREE.Material).dispose();
      this.livePointCloudObject = null;
      this.livePointCloudBounds = null;
      this.livePointCloudRevealCount = 0;
      this.livePointCloudTargetCount = 0;
      return;
    }

    if (kind === "playbackMap" && this.playbackMapPointCloudObject) {
      this.playbackMapPointCloudLayer.remove(this.playbackMapPointCloudObject);
      this.playbackMapPointCloudObject.geometry.dispose();
      (this.playbackMapPointCloudObject.material as THREE.Material).dispose();
      this.playbackMapPointCloudObject = null;
      return;
    }

    if (kind === "playbackScan" && this.playbackScanPointCloudObject) {
      this.playbackScanPointCloudLayer.remove(this.playbackScanPointCloudObject);
      this.playbackScanPointCloudObject.geometry.dispose();
      (this.playbackScanPointCloudObject.material as THREE.Material).dispose();
      this.playbackScanPointCloudObject = null;
    }
  }

  private shouldLoadGaussian(): boolean {
    const gaussian = this.manifest?.gaussian;
    return this.mode === "gs" &&
      this.visibility.gaussian &&
      Boolean(gaussian?.url || gaussian?.chunks?.length);
  }

  private async ensureDeferredAssets(): Promise<void> {
    const gaussian = this.manifest?.gaussian;
    if (!gaussian || !this.shouldLoadGaussian()) {
      return;
    }
    const sourceKey = getGaussianSourceKey(gaussian);
    if (!sourceKey) {
      return;
    }

    if (this.gaussianSourceKey === sourceKey) {
      return;
    }

    if (this.gaussianLoadingKey === sourceKey && this.gaussianLoadTask) {
      if (gaussian.format === "gs-chunks") {
        return;
      }
      await this.gaussianLoadTask;
      return;
    }

    this.gaussianLoadingKey = sourceKey;
    const generation = this.gaussianLoadGeneration;
    if (gaussian.format === "gs-chunks" && gaussian.chunks?.length) {
      this.gaussianLoadTask = this.loadGaussianChunks(gaussian.chunks, sourceKey, generation).finally(() => {
        if (this.gaussianLoadingKey === sourceKey) {
          this.gaussianLoadingKey = null;
        }
        this.gaussianLoadTask = null;
      });
      return;
    }

    if (!gaussian.url) {
      return;
    }

    this.gaussianLoadTask = this.loadSingleGaussian(gaussian.url, sourceKey, generation).finally(() => {
      if (this.gaussianLoadingKey === sourceKey) {
        this.gaussianLoadingKey = null;
      }
      this.gaussianLoadTask = null;
    });
    await this.gaussianLoadTask;
  }

  private async prefetchGaussianAssets(manifest: SceneManifest): Promise<void> {
    const gaussian = manifest.gaussian;
    if (!gaussian || this.gaussianPrefetchTask) {
      return;
    }

    const urls =
      gaussian.format === "gs-chunks" && gaussian.chunks?.length
        ? orderGaussianChunks(gaussian.chunks, this.camera.position, combineChunkBounds(gaussian.chunks)).map(
            (chunk) => chunk.url
          )
        : gaussian.url
          ? [gaussian.url]
          : [];

    const pending = urls.filter((url) => !this.prefetchedGaussianUrls.has(url)).slice(0, 10);
    if (!pending.length) {
      return;
    }

    this.gaussianPrefetchTask = (async () => {
      for (const url of pending) {
        try {
          await fetch(url, { cache: "force-cache" });
          this.prefetchedGaussianUrls.add(url);
        } catch (error) {
          console.warn("Failed to prefetch gaussian asset.", url, error);
          break;
        }
      }
    })().finally(() => {
      this.gaussianPrefetchTask = null;
    });
  }
}

function createRobotMarker(): THREE.Object3D {
  const group = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(0.22, 0.22, 0.18, 24),
    new THREE.MeshStandardMaterial({
      color: "#ff784f",
      metalness: 0.12,
      roughness: 0.62
    })
  );
  body.rotation.x = Math.PI / 2;
  body.position.z = 0.1;

  const heading = new THREE.Mesh(
    new THREE.ConeGeometry(0.12, 0.32, 18),
    new THREE.MeshStandardMaterial({
      color: "#ffd166",
      metalness: 0.05,
      roughness: 0.45
    })
  );
  heading.rotation.x = Math.PI / 2;
  heading.position.set(0.24, 0, 0.1);
  heading.rotation.z = -Math.PI / 2;

  group.add(body);
  group.add(heading);
  return group;
}

function createTrajectoryPoseMarker(color: THREE.Color): THREE.Object3D {
  const positions = [
    0, 0, 0,
    0.22, 0.12, 0.08,
    0, 0, 0,
    0.22, -0.12, 0.08,
    0, 0, 0,
    0.22, -0.12, -0.08,
    0, 0, 0,
    0.22, 0.12, -0.08,
    0.22, 0.12, 0.08,
    0.22, -0.12, 0.08,
    0.22, -0.12, 0.08,
    0.22, -0.12, -0.08,
    0.22, -0.12, -0.08,
    0.22, 0.12, -0.08,
    0.22, 0.12, -0.08,
    0.22, 0.12, 0.08
  ];

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: true,
    opacity: 0.82,
    depthWrite: false
  });
  return new THREE.LineSegments(geometry, material);
}

function createPointSpriteTexture(): THREE.Texture {
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const context = canvas.getContext("2d");
  if (!context) {
    return new THREE.Texture();
  }

  context.clearRect(0, 0, size, size);
  const gradient = context.createRadialGradient(
    size * 0.5,
    size * 0.5,
    size * 0.06,
    size * 0.5,
    size * 0.5,
    size * 0.5
  );
  gradient.addColorStop(0, "rgba(255,255,255,1)");
  gradient.addColorStop(0.72, "rgba(255,255,255,0.96)");
  gradient.addColorStop(1, "rgba(255,255,255,0)");
  context.fillStyle = gradient;
  context.beginPath();
  context.arc(size * 0.5, size * 0.5, size * 0.5, 0, Math.PI * 2);
  context.fill();

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;
  return texture;
}

function isFiniteBox(box: THREE.Box3): boolean {
  return Number.isFinite(box.min.x) &&
    Number.isFinite(box.min.y) &&
    Number.isFinite(box.min.z) &&
    Number.isFinite(box.max.x) &&
    Number.isFinite(box.max.y) &&
    Number.isFinite(box.max.z) &&
    box.max.x >= box.min.x &&
    box.max.y >= box.min.y &&
    box.max.z >= box.min.z;
}

function getGaussianSourceKey(source: SceneManifest["gaussian"] | undefined): string | null {
  if (!source) {
    return null;
  }
  if (source.format === "gs-chunks" && source.chunks?.length) {
    return `chunks:${source.chunks.map((chunk) => chunk.url).join("|")}`;
  }
  if (source.url) {
    return `${source.format}:${source.url}`;
  }
  return null;
}

function boxFromChunkBounds(bounds: NonNullable<GaussianChunk["bounds"]>): THREE.Box3 {
  return new THREE.Box3(
    new THREE.Vector3(bounds.min.x, bounds.min.y, bounds.min.z),
    new THREE.Vector3(bounds.max.x, bounds.max.y, bounds.max.z)
  );
}

function compareChunks(
  left: GaussianChunk,
  right: GaussianChunk,
  cameraPosition: THREE.Vector3,
  sceneCenter: THREE.Vector3
): number {
  const leftSplats = left.splats ?? 0;
  const rightSplats = right.splats ?? 0;
  if (leftSplats !== rightSplats) {
    return rightSplats - leftSplats;
  }

  const leftCenter = left.center ?? centerFromBounds(left.bounds);
  const rightCenter = right.center ?? centerFromBounds(right.bounds);
  const leftDistance = leftCenter
    ? cameraPosition.distanceToSquared(new THREE.Vector3(leftCenter.x, leftCenter.y, leftCenter.z))
    : Number.POSITIVE_INFINITY;
  const rightDistance = rightCenter
    ? cameraPosition.distanceToSquared(new THREE.Vector3(rightCenter.x, rightCenter.y, rightCenter.z))
    : Number.POSITIVE_INFINITY;
  const leftSceneDistance = leftCenter
    ? sceneCenter.distanceToSquared(new THREE.Vector3(leftCenter.x, leftCenter.y, leftCenter.z))
    : Number.POSITIVE_INFINITY;
  const rightSceneDistance = rightCenter
    ? sceneCenter.distanceToSquared(new THREE.Vector3(rightCenter.x, rightCenter.y, rightCenter.z))
    : Number.POSITIVE_INFINITY;

  if (leftSceneDistance !== rightSceneDistance) {
    return leftSceneDistance - rightSceneDistance;
  }

  if (leftDistance !== rightDistance) {
    return leftDistance - rightDistance;
  }

  return 0;
}

function centerFromBounds(bounds: GaussianChunk["bounds"]): { x: number; y: number; z: number } | null {
  if (!bounds) {
    return null;
  }
  return {
    x: (bounds.min.x + bounds.max.x) * 0.5,
    y: (bounds.min.y + bounds.max.y) * 0.5,
    z: (bounds.min.z + bounds.max.z) * 0.5
  };
}

function combineChunkBounds(chunks: GaussianChunk[]): THREE.Box3 | null {
  const validBounds = chunks
    .map((chunk) => (chunk.bounds ? boxFromChunkBounds(chunk.bounds) : null))
    .filter((box): box is THREE.Box3 => Boolean(box && isFiniteBox(box)));

  if (!validBounds.length) {
    return null;
  }

  const combined = validBounds[0].clone();
  for (let index = 1; index < validBounds.length; index += 1) {
    combined.union(validBounds[index]);
  }
  return combined;
}

function orderGaussianChunks(
  chunks: GaussianChunk[],
  cameraPosition: THREE.Vector3,
  combinedBounds: THREE.Box3 | null
): GaussianChunk[] {
  const sceneCenter = combinedBounds?.getCenter(new THREE.Vector3()) ?? new THREE.Vector3();
  return [...chunks].sort((left, right) => compareChunks(left, right, cameraPosition, sceneCenter));
}

function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if ("geometry" in child && child.geometry instanceof THREE.BufferGeometry) {
      child.geometry.dispose();
    }
    if ("material" in child) {
      const material = child.material;
      if (Array.isArray(material)) {
        material.forEach((item) => item.dispose());
      } else if (material instanceof THREE.Material) {
        material.dispose();
      }
    }
  });
}

function computePointCloudBounds(positions: Float32Array): THREE.Box3 | null {
  if (!positions.length) {
    return null;
  }

  const box = new THREE.Box3();
  const point = new THREE.Vector3();
  let hasPoint = false;

  for (let index = 0; index < positions.length; index += 3) {
    point.set(positions[index], positions[index + 1], positions[index + 2]);
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z)) {
      continue;
    }
    if (!hasPoint) {
      box.min.copy(point);
      box.max.copy(point);
      hasPoint = true;
    } else {
      box.expandByPoint(point);
    }
  }

  return hasPoint ? box : null;
}

function samplePointCloudForRender(cloud: DecodedPointCloud, maxPoints: number): DecodedPointCloud {
  if (cloud.renderedPointCount <= maxPoints) {
    return cloud;
  }

  const stride = Math.max(1, Math.ceil(cloud.renderedPointCount / maxPoints));
  const sampleCount = Math.ceil(cloud.renderedPointCount / stride);
  const positions = new Float32Array(sampleCount * 3);
  const colors =
    cloud.colors && cloud.colors.length === cloud.positions.length
      ? new Float32Array(sampleCount * 3)
      : undefined;

  let renderIndex = 0;
  for (let pointIndex = 0; pointIndex < cloud.renderedPointCount; pointIndex += stride) {
    const sourceOffset = pointIndex * 3;
    const targetOffset = renderIndex * 3;
    positions[targetOffset] = cloud.positions[sourceOffset];
    positions[targetOffset + 1] = cloud.positions[sourceOffset + 1];
    positions[targetOffset + 2] = cloud.positions[sourceOffset + 2];

    if (colors && cloud.colors) {
      colors[targetOffset] = cloud.colors[sourceOffset];
      colors[targetOffset + 1] = cloud.colors[sourceOffset + 1];
      colors[targetOffset + 2] = cloud.colors[sourceOffset + 2];
    }
    renderIndex += 1;
  }

  return {
    frameId: cloud.frameId,
    stampMs: cloud.stampMs,
    sourcePointCount: cloud.sourcePointCount,
    renderedPointCount: renderIndex,
    positions: renderIndex === sampleCount ? positions : positions.slice(0, renderIndex * 3),
    colors:
      colors && renderIndex === sampleCount
        ? colors
        : colors
          ? colors.slice(0, renderIndex * 3)
          : undefined
  };
}

function ensureGeometryVertexColors(geometry: THREE.BufferGeometry): boolean {
  const existingColors = geometry.getAttribute("color");
  if (existingColors && existingColors.count > 0) {
    return true;
  }

  const position = geometry.getAttribute("position");
  if (!(position instanceof THREE.BufferAttribute) || position.count === 0) {
    return false;
  }

  const bounds = geometry.boundingBox ?? new THREE.Box3().setFromBufferAttribute(position);
  const zMin = bounds.min.z;
  const zMax = bounds.max.z;
  const zRange = Math.max(1e-5, zMax - zMin);
  const colors = new Float32Array(position.count * 3);
  const color = new THREE.Color();

  for (let index = 0; index < position.count; index += 1) {
    const z = position.getZ(index);
    const t = THREE.MathUtils.clamp((z - zMin) / zRange, 0, 1);
    color.setHSL(0.62 - t * 0.54, 0.78, 0.54);
    const offset = index * 3;
    colors[offset] = color.r;
    colors[offset + 1] = color.g;
    colors[offset + 2] = color.b;
  }

  geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  return true;
}
