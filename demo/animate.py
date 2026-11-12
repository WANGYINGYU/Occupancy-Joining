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
from relations import Relations


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


def panel(canvas, box, title, bounds, points=None, grid=None):
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


def grid_corners(grid):
    y, x = np.nonzero(grid['counts'] > 0)
    if not len(x):
        raise ValueError('Cannot animate an unobserved map')
    lo = grid['origin'] + (np.array([x.min(), y.min()]) - .5) * grid['resolution']
    hi = grid['origin'] + (np.array([x.max(), y.max()]) + .5) * grid['resolution']
    return np.array([[lo[0], lo[1]], [lo[0], hi[1]], [hi[0], lo[1]], [hi[0], hi[1]]])


def whole_bounds(locals_, history, grids):
    corners = [grid_corners(grid) for grid in locals_]
    points = [np.concatenate(transformed(corners, poses)) for poses in history]
    points += [grid_corners(grid) for grid in grids]
    points = np.concatenate(points)
    padding = max(.5, np.max(np.ptp(points, axis=0)) * .04)
    lo, hi = points.min(axis=0) - padding, points.max(axis=0) + padding
    return [lo[0], hi[0], lo[1], hi[1]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
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
    bounds = whole_bounds(locals_, poses, grids)
    errors = evaluation_rmse(result, cfg['submaps'], poses)
    relation_errors = None
    if (result / 'reference_relations.txt').is_file():
        reference = Relations(result / 'reference_timestamps.txt', result / 'reference_relations.txt')
        relation_errors = reference.history(np.loadtxt(result / 'input_poses.txt'), cfg['submaps'], poses)
        saved = json.loads((result / 'relative_pose_metrics.json').read_text())
        for actual, state in [(relation_errors[0], 'initial'), (relation_errors[-1], 'optimized')]:
            for key in ['translation_m', 'rotation_deg']:
                if not np.isclose(actual[key]['rmse'], saved[state][key]['rmse'], rtol=1e-10, atol=1e-12):
                    raise ValueError('Animation relative-pose metric differs from the saved evaluation')
    box = (24, 94, 1152, 628)

    def canvas_for():
        canvas = Image.new('RGB', (1200, 780), 'white')
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 14), args.title, fill=(0, 0, 0), font=font(26))
        draw.text((24, 51), args.subtitle, fill=(80, 80, 80), font=font(16))
        return canvas, draw

    frames, durations = [], []
    selected = np.unique(np.linspace(0, len(steps) - 1, min(args.frames, len(steps)), dtype=int))
    for index in selected:
        title = 'Before joining' if index == 0 else f'Joining: recorded update {steps[index]} / {steps[-1]}'
        canvas, draw = canvas_for()
        panel(canvas, box, title, bounds, points=transformed(occupied, poses[index]))
        footer = f'{len(locals_)} local submaps | fixed whole-map view'
        if errors is not None:
            footer += f' | Pose RMSE (evaluation GT): {distance_label(errors[index])}'
        if relation_errors is not None:
            metric = relation_errors[index]
            footer += f' | Relation RMSE: {metric["translation_m"]["rmse"]:.3f} m / {metric["rotation_deg"]["rmse"]:.3f} deg'
        draw.text((24, 746), footer, fill=(35, 35, 35), font=font(18))
        frames.append(indexed(canvas))
        durations.append(2000 if index == 0 else (2500 if index == selected[-1] else 120))
    for grid, title in zip(grids, ['Initial occupancy map', 'Direct optimized occupancy field', 'Map rebuilt with optimized poses']):
        canvas, draw = canvas_for()
        panel(canvas, box, title, bounds, grid=grid)
        footer = f'{grid["resolution"]:g} m grid | black occupied, white free, gray unknown'
        draw.text((24, 746), footer, fill=(35, 35, 35), font=font(18))
        frames.append(indexed(canvas))
        durations.append(2200)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, optimize=False, disposal=1)
    print(f'Animation: {args.output} ({len(frames)} frames)', flush=True)


if __name__ == '__main__':
    main()
