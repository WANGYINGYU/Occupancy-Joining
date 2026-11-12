#!/usr/bin/env python3
"""Render recorded submap updates and the three output maps as an animated GIF."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/occupancy_joining/scripts'))
from grid_io import read_grid, probabilities


def font(size):
    try:
        return ImageFont.truetype('DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default(size=size)


def manifest_path(result):
    for path in [result / 'submaps.yaml', result.parent / 'prepared/submaps.yaml']:
        if path.is_file():
            return path
    command = json.loads((result / 'backend_command.json').read_text())
    return Path(command[command.index('--manifest') + 1])


def occupied_points(grid):
    observed = grid['counts'] > 0
    mean = np.divide(grid['values'], grid['counts'], out=np.zeros_like(grid['values']), where=observed)
    y, x = np.nonzero(observed & (mean >= math.log(.65 / .35)))
    return np.column_stack((x, y)) * grid['resolution'] + grid['origin']


def transformed(points, poses):
    result = []
    for cells, pose in zip(points, poses):
        c, s = np.cos(pose[2]), np.sin(pose[2])
        result.append(cells @ np.array([[c, s], [-s, c]]) + pose[:2])
    return np.concatenate(result)


def panel(canvas, box, title, bounds, points=None, grid=None, roi=None):
    x, y, width, height = box
    draw = ImageDraw.Draw(canvas)
    draw.text((x, y), title, fill=30, font=font(17))
    xmin, xmax, ymin, ymax = bounds
    scale = min((width - 12) / (xmax - xmin), (height - 36) / (ymax - ymin))
    w, h = max(1, round(scale * (xmax - xmin))), max(1, round(scale * (ymax - ymin)))
    left, top = x + (width - w) // 2, y + 30 + (height - 36 - h) // 2
    pixels = np.full((h, w), 255, dtype=np.uint8)
    if points is not None:
        ix = np.floor((points[:, 0] - xmin) / (xmax - xmin) * w).astype(int)
        iy = np.floor((ymax - points[:, 1]) / (ymax - ymin) * h).astype(int)
        good = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        pixels[iy[good], ix[good]] = 0
    else:
        xs = xmin + (np.arange(w) + .5) / w * (xmax - xmin)
        ys = ymax - (np.arange(h) + .5) / h * (ymax - ymin)
        ix = np.floor((xs - grid['origin'][0]) / grid['resolution'] + .5).astype(int)
        iy = np.floor((ys - grid['origin'][1]) / grid['resolution'] + .5).astype(int)
        vx, vy = (ix >= 0) & (ix < grid['width']), (iy >= 0) & (iy < grid['height'])
        pixels.fill(128)
        pixels[np.ix_(vy, vx)] = grid['gray'][np.ix_(iy[vy], ix[vx])]
    canvas.paste(Image.fromarray(pixels), (left, top))
    draw.rectangle((left, top, left + w, top + h), outline=210)
    if roi is not None:
        a, b, c, d = roi
        draw.rectangle((left + (a - xmin) * scale, top + (ymax - d) * scale,
                        left + (b - xmin) * scale, top + (ymax - c) * scale), outline=100, width=2)


def indexed(canvas):
    image = canvas.point([min(31, (value + 4) // 8) for value in range(256)], 'P')
    palette = []
    for value in range(256):
        level = 255 if value >= 31 else value * 8
        palette.extend([level] * 3)
    image.putpalette(palette)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--roi', nargs=4, type=float, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    parser.add_argument('--title', default='Grid-based Submap Joining')
    parser.add_argument('--frames', type=int, default=40)
    args = parser.parse_args()
    if args.frames < 2:
        parser.error('--frames must be at least 2')
    result = args.result.resolve(strict=True)
    manifest = manifest_path(result)
    cfg = yaml.safe_load(manifest.read_text())
    locals_ = []
    for item in cfg['submaps']:
        path = Path(item['grid'])
        locals_.append(read_grid(path if path.is_absolute() else manifest.parent / path))
    history = np.loadtxt(result / 'maps/pose_history.csv', delimiter=',', skiprows=1, ndmin=2)
    steps = np.unique(history[:, 0]).astype(int)
    poses = []
    for step in steps:
        rows = history[history[:, 0] == step]
        if not np.array_equal(rows[:, 1], np.arange(len(locals_))):
            raise ValueError('Each recorded update must contain every submap exactly once')
        poses.append(rows[:, 2:5])
    for actual, state in [(poses[0], 'initial'), (poses[-1], 'optimized')]:
        if not np.array_equal(actual, np.loadtxt(result / f'maps/{state}/submap_poses.txt', ndmin=2)):
            raise ValueError('Pose history endpoints differ from the exported states')
    grids = []
    for state in ['initial', 'optimized', 'reintegrated']:
        folder = result / 'maps' / state
        grid = read_grid(folder / ('display.grid' if (folder / 'display.grid').exists() else 'map.grid'))
        grid['gray'] = np.rint(255 * (1 - probabilities(grid))).astype(np.uint8)
        grid['gray'][grid['counts'] <= 0] = 128
        grids.append(grid)
    occupied = [occupied_points(grid) for grid in locals_]
    initial = transformed(occupied, poses[0])
    all_ends = np.concatenate([initial, transformed(occupied, poses[-1])])
    lo, hi = all_ends.min(axis=0) - 3, all_ends.max(axis=0) + 3
    whole = [lo[0], hi[0], lo[1], hi[1]]
    roi = args.roi or [(lo[0] * 3 + hi[0]) / 4, (lo[0] + hi[0] * 3) / 4,
                       (lo[1] * 3 + hi[1]) / 4, (lo[1] + hi[1] * 3) / 4]
    if not np.isfinite(roi).all() or roi[0] >= roi[1] or roi[2] >= roi[3]:
        parser.error('--roi requires finite, increasing bounds')
    frames, durations = [], []
    selected = np.unique(np.linspace(0, len(steps) - 1, min(args.frames, len(steps)), dtype=int))
    for index in selected:
        canvas = Image.new('L', (1200, 860), 255)
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 14), args.title, fill=0, font=font(26))
        draw.text((24, 51), f'{len(locals_)} submaps | occupied cells (p >= 0.65) | recorded updates', fill=80, font=font(16))
        current = transformed(occupied, poses[index])
        for x, title, points in [(24, 'Before joining', initial), (620, f'Joining: update {steps[index]} / {steps[-1]}', current)]:
            panel(canvas, (x, 90, 556, 440), title, whole, points=points, roi=roi)
            panel(canvas, (x, 550, 556, 275), 'Central detail', roi, points=points)
        frames.append(indexed(canvas))
        durations.append(1600 if index == 0 else 120)
    canvas = Image.new('L', (1200, 860), 255)
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 14), args.title, fill=0, font=font(26))
    draw.text((24, 51), 'Final occupancy maps', fill=80, font=font(16))
    for i, (grid, title) in enumerate(zip(grids, ['Initial', 'Optimized field', 'Rebuilt map'])):
        x = 18 + i * 398
        panel(canvas, (x, 92, 376, 440), title, whole, grid=grid, roi=roi)
        panel(canvas, (x, 550, 376, 275), f'Central detail | {grid["resolution"]:g} m', roi, grid=grid)
    frames.append(indexed(canvas)); durations.append(4500)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, optimize=False, disposal=1)
    print(f'Animation: {args.output} ({len(frames)} frames)', flush=True)


if __name__ == '__main__':
    main()
