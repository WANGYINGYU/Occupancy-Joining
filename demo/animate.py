#!/usr/bin/env python3
"""Render recorded submap updates and the three output maps as an animated GIF."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/occupancy_joining/scripts'))
from grid_io import read_grid, probabilities
from geometry import se2, pose2


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
    return result


def panel(canvas, box, title, bounds, points=None, grid=None, roi=None):
    x, y, width, height = box
    draw = ImageDraw.Draw(canvas)
    draw.text((x, y), title, fill=(30, 30, 30), font=font(17))
    xmin, xmax, ymin, ymax = bounds
    scale = min((width - 12) / (xmax - xmin), (height - 36) / (ymax - ymin))
    w, h = max(1, round(scale * (xmax - xmin))), max(1, round(scale * (ymax - ymin)))
    left, top = x + (width - w) // 2, y + 30 + (height - 36 - h) // 2
    pixels = np.full((h, w, 3), 255, dtype=np.uint8)
    if points is not None:
        colors = [(35, 110, 190), (225, 100, 30), (0, 155, 120), (155, 65, 160)]
        for i, group in enumerate(points):
            ix = np.floor((group[:, 0] - xmin) / (xmax - xmin) * w).astype(int)
            iy = np.floor((ymax - group[:, 1]) / (ymax - ymin) * h).astype(int)
            good = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            mask = np.zeros((h, w), dtype=np.uint8)
            mask[iy[good], ix[good]] = 255
            mask = np.asarray(Image.fromarray(mask).filter(ImageFilter.MaxFilter(3))) > 0
            pixels[mask] = np.minimum(pixels[mask], colors[i % len(colors)])
    else:
        xs = xmin + (np.arange(w) + .5) / w * (xmax - xmin)
        ys = ymax - (np.arange(h) + .5) / h * (ymax - ymin)
        ix = np.floor((xs - grid['origin'][0]) / grid['resolution'] + .5).astype(int)
        iy = np.floor((ys - grid['origin'][1]) / grid['resolution'] + .5).astype(int)
        vx, vy = (ix >= 0) & (ix < grid['width']), (iy >= 0) & (iy < grid['height'])
        pixels.fill(128)
        pixels[np.ix_(vy, vx)] = grid['gray'][np.ix_(iy[vy], ix[vx])][..., None]
    canvas.paste(Image.fromarray(pixels), (left, top))
    draw.rectangle((left, top, left + w, top + h), outline=(210, 210, 210))
    if roi is not None:
        a, b, c, d = roi
        draw.rectangle((left + (a - xmin) * scale, top + (ymax - d) * scale,
                        left + (b - xmin) * scale, top + (ymax - c) * scale), outline=(100, 100, 100), width=2)


def indexed(canvas):
    # A fixed palette prevents color flicker between recorded updates.
    palette = Image.new('P', (1, 1))
    colors = [(35, 110, 190), (225, 100, 30), (0, 155, 120), (155, 65, 160)]
    entries = [(v, v, v) for v in sorted(set(range(0, 256, 8)) | {30, 80, 100, 128, 210, 255})]
    for mask in range(1, 16):
        entries.append(tuple(min(colors[i][k] for i in range(4) if mask & (1 << i)) for k in range(3)))
    entries += [(255, 255, 255)] * (256 - len(entries))
    palette.putpalette([channel for color in entries for channel in color])
    return canvas.quantize(palette=palette, dither=Image.Dither.NONE)


def evaluation_rmse(result, items, history):
    if not (result / 'groundtruth.txt').is_file() or not (result / 'input_poses.txt').is_file():
        return None
    inputs = np.loadtxt(result / 'input_poses.txt', ndmin=2)
    truth = np.loadtxt(result / 'groundtruth.txt', ndmin=2)
    gauge = se2(inputs[0]) @ np.linalg.inv(se2(truth[0]))
    target = np.asarray([pose2(gauge @ se2(p)) for p in truth])
    blocks = []
    for item in items:
        anchor = inputs[item['anchor_index']]
        blocks.append(np.asarray([pose2(np.linalg.inv(se2(anchor)) @ se2(p))
                                  for p in inputs[item['begin']:item['end']]]))
    values = []
    for anchors in history:
        robot = np.zeros_like(inputs)
        written = np.zeros(len(inputs), dtype=bool)
        for anchor, local, item in zip(anchors, blocks, items):
            for j, point in enumerate(local, start=item['begin']):
                if not written[j]:
                    robot[j] = pose2(se2(anchor) @ se2(point))
                    written[j] = True
        if not written.all():
            raise ValueError('Incomplete evaluation trajectory')
        values.append(float(np.sqrt(np.mean(np.sum((robot[1:, :2] - target[1:, :2]) ** 2, axis=1)))))
    metrics = json.loads((result / 'metrics.json').read_text())
    for value, state in [(values[0], 'initial'), (values[-1], 'optimized')]:
        if not np.isclose(value, metrics[state]['translation_rmse_m'], rtol=1e-10, atol=1e-12):
            raise ValueError('Animation RMSE differs from the saved evaluation')
    return values


def distance_label(value):
    return '< 0.001 m' if value < .001 else f'{value:.3f} m'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--roi', nargs=4, type=float, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    parser.add_argument('--title', default='Grid-based Submap Joining')
    parser.add_argument('--subtitle', default='Local submap occupied cells | recorded optimizer updates')
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
    all_ends = np.concatenate(initial + transformed(occupied, poses[-1]))
    padding = max(.5, np.max(np.ptp(all_ends, axis=0)) * .04)
    lo, hi = all_ends.min(axis=0) - padding, all_ends.max(axis=0) + padding
    whole = [lo[0], hi[0], lo[1], hi[1]]
    roi = args.roi or [(lo[0] * 3 + hi[0]) / 4, (lo[0] + hi[0] * 3) / 4,
                       (lo[1] * 3 + hi[1]) / 4, (lo[1] + hi[1] * 3) / 4]
    if not np.isfinite(roi).all() or roi[0] >= roi[1] or roi[2] >= roi[3]:
        parser.error('--roi requires finite, increasing bounds')
    errors = evaluation_rmse(result, cfg['submaps'], poses)
    frames, durations = [], []
    selected = np.unique(np.linspace(0, len(steps) - 1, min(args.frames, len(steps)), dtype=int))
    for index in selected:
        canvas = Image.new('RGB', (1200, 860), 'white')
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 14), args.title, fill=(0, 0, 0), font=font(26))
        draw.text((24, 51), args.subtitle, fill=(80, 80, 80), font=font(16))
        for i, color in enumerate([(35, 110, 190), (225, 100, 30), (0, 155, 120)][:min(3, len(locals_))]):
            draw.text((24 + i * 125, 73), f'Submap {i + 1}', fill=color, font=font(14))
        current = transformed(occupied, poses[index])
        for x, title, points in [(24, 'Before joining', initial), (620, f'Joining: update {steps[index]} / {steps[-1]}', current)]:
            panel(canvas, (x, 90, 556, 440), title, whole, points=points, roi=roi)
            panel(canvas, (x, 550, 556, 275), 'Central detail', roi, points=points)
        if errors is not None:
            draw.text((24, 832), f'Initial pose RMSE: {distance_label(errors[0])}', fill=(35, 35, 35), font=font(18))
            draw.text((620, 832), f'Current pose RMSE: {distance_label(errors[index])}', fill=(35, 35, 35), font=font(18))
        frames.append(indexed(canvas))
        durations.append(2000 if index == 0 else (3200 if index == selected[-1] else max(180, 2500 // len(selected))))
    canvas = Image.new('RGB', (1200, 860), 'white')
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 14), args.title, fill=(0, 0, 0), font=font(26))
    draw.text((24, 51), 'Occupancy maps | same world frame and view | black occupied, white free, gray unknown', fill=(80, 80, 80), font=font(16))
    for i, (grid, title) in enumerate(zip(grids, ['Initial', 'Optimized field', 'Rebuilt map'])):
        x = 18 + i * 398
        panel(canvas, (x, 92, 376, 440), title, whole, grid=grid, roi=roi)
        panel(canvas, (x, 550, 376, 275), f'Central detail | {grid["resolution"]:g} m', roi, grid=grid)
    if errors is not None:
        draw.text((24, 832), f'Pose RMSE (evaluation GT): {distance_label(errors[0])} -> {distance_label(errors[-1])}', fill=(35, 35, 35), font=font(18))
    frames.append(indexed(canvas)); durations.append(5000)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, optimize=False, disposal=1)
    print(f'Animation: {args.output} ({len(frames)} frames)', flush=True)


if __name__ == '__main__':
    main()
