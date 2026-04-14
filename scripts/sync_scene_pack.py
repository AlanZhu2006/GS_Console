#!/usr/bin/env python3
import argparse
import json
import os
import re
import shutil
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Sync GS-SDF output into a web scene pack.")
    parser.add_argument("--output-dir", required=True, help="GS-SDF output directory")
    parser.add_argument("--scene-config", help="Host-side scene config yaml")
    parser.add_argument(
        "--scene-root",
        default="/home/chatsign/gs-sdf/examples/web-ui/public/scenes",
        help="Web scene root directory",
    )
    parser.add_argument("--scene-id", help="Override scene id")
    parser.add_argument("--url-prefix", default="/scenes", help="Public URL prefix")
    parser.add_argument("--status", default="running", help="training status label")
    parser.add_argument("--robot-radius", type=float, default=0.28)
    parser.add_argument("--robot-height", type=float, default=0.45)
    return parser.parse_args()


def find_text(path: Path, pattern: str):
    if not path.is_file():
        return None
    text = path.read_text()
    match = re.search(pattern, text, re.MULTILINE)
    return match.group(1) if match else None


def parse_scene_meta(config_path: Path):
    meta = {
        "leaf_size": None,
        "map_origin": {"x": 0.0, "y": 0.0, "z": 0.0},
        "map_size": None,
    }
    if not config_path or not config_path.is_file():
        return meta

    text = config_path.read_text()

    leaf_match = re.search(r"leaf_sizes:\s*([0-9.+-eE]+)", text)
    if leaf_match:
        meta["leaf_size"] = float(leaf_match.group(1))

    map_size_match = re.search(r"map_size:\s*([0-9.+-eE]+)", text)
    if map_size_match:
        meta["map_size"] = float(map_size_match.group(1))

    origin_match = re.search(
        r"map_origin:.*?data:\s*\[\s*([0-9.+-eE]+)\s*,\s*([0-9.+-eE]+)\s*,\s*([0-9.+-eE]+)\s*\]",
        text,
        re.DOTALL,
    )
    if origin_match:
        meta["map_origin"] = {
            "x": float(origin_match.group(1)),
            "y": float(origin_match.group(2)),
            "z": float(origin_match.group(3)),
        }

    return meta


def parse_camera_config(config_path: Path):
    camera = {
        "width": None,
        "height": None,
        "fx": None,
        "fy": None,
        "cx": None,
        "cy": None,
        "tCL": None,
        "tBL": None,
    }
    if not config_path or not config_path.is_file():
        return camera

    text = config_path.read_text()
    for key in ("width", "height", "fx", "fy", "cx", "cy"):
        match = re.search(rf"{key}:\s*([0-9.+-eE]+)", text)
        if match:
            camera[key] = float(match.group(1))

    def parse_matrix(name: str):
        match = re.search(
            rf"{name}:\s*!!opencv-matrix.*?data:\s*\[(.*?)\]",
            text,
            re.DOTALL,
        )
        if not match:
            return None
        values = [float(token.strip()) for token in match.group(1).replace("\n", " ").split(",") if token.strip()]
        if len(values) != 16:
            return None
        return values

    camera["tCL"] = parse_matrix("T_C_L")
    camera["tBL"] = parse_matrix("T_B_L")
    return camera


def safe_link(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.is_symlink() or dst.exists():
        if dst.resolve() == src.resolve():
            return
        if dst.is_dir() and not dst.is_symlink():
            shutil.rmtree(dst)
        else:
            dst.unlink()
    dst.symlink_to(src)


def first_match(root: Path, patterns):
    for pattern in patterns:
        matches = sorted(root.glob(pattern))
        if matches:
            return matches[0]
    return None


def load_json(path: Path):
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def format_variant_label(variant: str):
    return variant.replace("_", " ").replace("-", " ").title()


def build_chunk_source(chunks_manifest, args, scene_id: str, model_root: Path, variant: str):
    chunk_dir_name = "gs_chunks" if variant == "balanced" else f"gs_chunks_{variant}"
    chunk_scene_root = model_root / chunk_dir_name
    if chunk_scene_root.exists() or chunk_scene_root.is_symlink():
        if chunk_scene_root.is_dir() and not chunk_scene_root.is_symlink():
            shutil.rmtree(chunk_scene_root)
        else:
            chunk_scene_root.unlink()
    chunk_scene_root.mkdir(parents=True, exist_ok=True)

    chunk_entries = []
    for chunk in chunks_manifest["chunks"]:
        src = Path(chunk["path"]).resolve()
        dst = chunk_scene_root / chunk["file"]
        safe_link(src, dst)
        chunk_entries.append(
            {
                "id": chunk.get("id"),
                "url": f"{args.url_prefix}/{scene_id}/model/{chunk_dir_name}/{chunk['file']}",
                "splats": chunk.get("splats"),
                "bytes": chunk.get("bytes", src.stat().st_size),
                "center": chunk.get("center"),
                "bounds": chunk.get("bounds"),
            }
        )

    return {
        "format": "gs-chunks",
        "shDegree": int(chunks_manifest.get("output", {}).get("shDegree", 1)),
        "variant": variant,
        "label": format_variant_label(variant),
        "chunks": chunk_entries,
    }


def main():
    args = parse_args()
    output_dir = Path(args.output_dir).resolve()
    if not output_dir.is_dir():
        raise SystemExit(f"Output dir not found: {output_dir}")

    scene_id = args.scene_id or output_dir.name
    scene_root = Path(args.scene_root).resolve() / scene_id
    model_root = scene_root / "model"

    config_path = Path(args.scene_config).resolve() if args.scene_config else None
    if not config_path or not config_path.is_file():
      fallback = output_dir / "model" / "config" / "scene" / "config.yaml"
      if fallback.is_file():
          config_path = fallback

    meta = parse_scene_meta(config_path) if config_path else parse_scene_meta(Path())
    camera = parse_camera_config(config_path) if config_path else parse_camera_config(Path())

    repo_root = Path(__file__).resolve().parent.parent
    processed_model_root = repo_root / "runtime" / "processed" / output_dir.name / "model"
    gs_chunks_meta = processed_model_root / "gs_chunks.json"
    gs_runtime_spz = output_dir / "model" / "gs_runtime.spz"
    gs_runtime_meta = output_dir / "model" / "gs_runtime.json"
    if not gs_runtime_spz.is_file():
        processed_spz = processed_model_root / "gs_runtime.spz"
        processed_meta = processed_model_root / "gs_runtime.json"
        if processed_spz.is_file():
            gs_runtime_spz = processed_spz
        if processed_meta.is_file():
            gs_runtime_meta = processed_meta
    gs_ply = output_dir / "model" / "gs.ply"
    occ_ply = output_dir / "model" / "as_occ_prior.ply"
    train_points = output_dir / "train_points.ply"
    mesh_ply = first_match(output_dir, ["mesh*.ply", "model/mesh*.ply", "model/gs_*.ply"])

    scene_root.mkdir(parents=True, exist_ok=True)
    model_root.mkdir(parents=True, exist_ok=True)

    runtime_sh_degree = 1
    if gs_runtime_meta.is_file():
        try:
            runtime_meta = json.loads(gs_runtime_meta.read_text())
            runtime_sh_degree = int(runtime_meta.get("output", {}).get("shDegree", runtime_sh_degree))
        except Exception:
            runtime_sh_degree = 1

    manifest = {
        "sceneId": scene_id,
        "frameId": "world",
        "training": {
            "status": args.status,
            "outputDir": str(output_dir),
            "configPath": str(config_path) if config_path else None,
        },
        "robot": {
            "radius": args.robot_radius,
            "height": args.robot_height,
        },
        "meta": {
            "leafSize": meta["leaf_size"],
            "mapOrigin": meta["map_origin"],
            "mapSize": meta["map_size"],
        },
        "camera": camera,
        "assets": {},
    }

    gaussian_variants = {}
    if gs_chunks_meta.is_file():
        chunks_manifest = load_json(gs_chunks_meta)
        if chunks_manifest and chunks_manifest.get("chunks"):
            gaussian_variants["balanced"] = build_chunk_source(
                chunks_manifest, args, scene_id, model_root, "balanced"
            )

    for extra_meta in sorted(processed_model_root.glob("gs_chunks_*.json")):
        variant = extra_meta.stem.removeprefix("gs_chunks_")
        if not variant:
            continue
        chunks_manifest = load_json(extra_meta)
        if not chunks_manifest or not chunks_manifest.get("chunks"):
            continue
        gaussian_variants[variant] = build_chunk_source(
            chunks_manifest, args, scene_id, model_root, variant
        )

    if gaussian_variants:
        preferred_variants = ["quality", "balanced", "ultra", "fast"]
        default_variant = next(
            (variant for variant in preferred_variants if variant in gaussian_variants),
            sorted(gaussian_variants.keys())[0],
        )
        manifest["gaussian"] = gaussian_variants[default_variant]
        manifest["gaussianVariants"] = gaussian_variants
        manifest["assets"]["gaussian"] = "ready"
    elif gs_runtime_spz.is_file():
        dst = model_root / "gs_runtime.spz"
        safe_link(gs_runtime_spz, dst)
        manifest["gaussian"] = {
            "format": "spz",
            "url": f"{args.url_prefix}/{scene_id}/model/gs_runtime.spz",
            "shDegree": runtime_sh_degree,
            "variant": "balanced",
            "label": "Balanced",
        }
        manifest["assets"]["gaussian"] = "ready"
    elif gs_ply.is_file():
        dst = model_root / "gs.ply"
        safe_link(gs_ply, dst)
        manifest["gaussian"] = {
            "format": "gs-ply",
            "url": f"{args.url_prefix}/{scene_id}/model/gs.ply",
            "shDegree": 3,
            "variant": "raw",
            "label": "Raw",
        }
        manifest["assets"]["gaussian"] = "ready"
    else:
        manifest["assets"]["gaussian"] = "missing"

    if occ_ply.is_file():
        dst = model_root / "as_occ_prior.ply"
        safe_link(occ_ply, dst)
        manifest["occupancy"] = {
            "source": "as_occ_prior",
            "url": f"{args.url_prefix}/{scene_id}/model/as_occ_prior.ply",
            "resolution": meta["leaf_size"],
            "origin": {
                "x": meta["map_origin"]["x"],
                "y": meta["map_origin"]["y"],
            },
        }
        manifest["assets"]["occupancy"] = "ready"
    else:
        manifest["assets"]["occupancy"] = "missing"

    if mesh_ply and mesh_ply.is_file():
        dst = model_root / mesh_ply.name
        safe_link(mesh_ply, dst)
        manifest["mesh"] = {
            "format": "ply",
            "url": f"{args.url_prefix}/{scene_id}/model/{mesh_ply.name}",
        }
        manifest["assets"]["mesh"] = "ready"
    else:
        manifest["assets"]["mesh"] = "missing"

    if train_points.is_file():
        dst = model_root / "train_points.ply"
        safe_link(train_points, dst)
        manifest["rawPointCloud"] = {
            "format": "ply",
            "url": f"{args.url_prefix}/{scene_id}/model/train_points.ply",
        }
        manifest["assets"]["rawPointCloud"] = "ready"
    elif occ_ply.is_file():
        dst = model_root / "as_occ_prior.ply"
        safe_link(occ_ply, dst)
        manifest["rawPointCloud"] = {
            "format": "ply",
            "url": f"{args.url_prefix}/{scene_id}/model/as_occ_prior.ply",
        }
        manifest["assets"]["rawPointCloud"] = "fallback"
    else:
        manifest["assets"]["rawPointCloud"] = "missing"

    manifest_path = scene_root / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"sceneRoot": str(scene_root), "manifest": str(manifest_path), "assets": manifest["assets"]}, indent=2))


if __name__ == "__main__":
    main()
