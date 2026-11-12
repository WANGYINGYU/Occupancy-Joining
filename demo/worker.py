#!/usr/bin/env python3
"""Run a versioned TXT demo and export maps, independent metrics and animation."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from relations import Relations

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', choices=['room', 'intel'], required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.data = args.data.resolve(strict=True)
    args.output = args.output.resolve()
    started = time.monotonic()
    subprocess.run([sys.executable, str(ROOT / 'src/occupancy_joining/scripts/joining_pipeline.py'),
                    '--config', str(args.data / 'demo.yaml'), '--output', str(args.output)], check=True)
    states = [('initial', 'Initial'), ('optimized', 'Direct optimized map'),
              ('reintegrated', 'Rebuilt with optimized poses')]
    figures = []
    for state, title in states:
        folder = args.output / 'maps' / state
        image = f'maps/{state}/' + ('display/map.png' if (folder / 'display/map.png').exists() else 'map.png')
        figures.append(f'<section><h2>{title}</h2><a href="{image}"><img src="{image}" alt="{title}"></a></section>')
    if args.demo == 'intel':
        for source, target in [('timestamps.txt', 'reference_timestamps.txt'),
                               ('relations.txt', 'reference_relations.txt')]:
            shutil.copyfile(args.data / source, args.output / target)
        reference = Relations(args.output / 'reference_timestamps.txt', args.output / 'reference_relations.txt')
        metrics = {state: reference.score(np.loadtxt(args.output / f'maps/{state}/trajectory.txt'))
                   for state, _ in states}
        (args.output / 'relative_pose_metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
        rows = ''.join(f'<tr><td>{title}</td><td>{metrics[state]["translation_m"]["rmse"]:.3f}</td><td>{metrics[state]["rotation_deg"]["rmse"]:.3f}</td></tr>' for state, title in states)
        evaluation = f'<h2>Independent manual relative-pose references ({len(reference.indices):,} pairs)</h2><table><tr><th>State</th><th>Translation RMSE [m]</th><th>Rotation RMSE [deg]</th></tr>{rows}</table>'
        title = 'Grid-based Submap Joining | Intel Research Lab'
        note = 'Real indoor laser dataset | 13,631 scans | 14 submaps | local tracking initialization'
    else:
        metrics = json.loads((args.output / 'metrics.json').read_text())
        rows = ''.join(f'<tr><td>{title}</td><td>{metrics[state]["translation_rmse_m"]:.6f}</td><td>{metrics[state]["yaw_rmse_rad"]:.6f}</td></tr>' for state, title in states)
        evaluation = f'<h2>Synthetic GT evaluation</h2><table><tr><th>State</th><th>Translation RMSE [m]</th><th>Yaw RMSE [rad]</th></tr>{rows}</table>'
        title = 'Grid-based Submap Joining | Synthetic room'
        note = 'Synthetic room | three internally exact submaps with imposed frame errors'
    (args.output / 'index.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Occupancy Joining demo</title><style>body{font:16px/1.6 sans-serif;margin:24px}main{display:flex;gap:18px}section{flex:1;min-width:0}img{width:100%;height:auto}h2{font-size:18px}td,th{padding:6px 18px;text-align:left}</style><h1>' + title + '</h1><p>' + note + '</p><main>' + ''.join(figures) + '</main><p><a href="demo.gif">Animated submap joining</a></p>' + evaluation + '</html>')
    subprocess.run([sys.executable, str(ROOT / 'demo/animate.py'), '--result', str(args.output),
                    '--output', str(args.output / 'demo.gif'), '--title', title, '--subtitle', note], check=True)
    metadata = json.loads((ROOT / 'demo' / f'{args.demo}.json').read_text())
    (args.output / 'demo.json').write_text(json.dumps(dict(demo=args.demo, frames=metadata['frames'],
        seconds=time.monotonic()-started, optimizer_source='demo.yaml', display_resolution=.05), indent=2) + '\n')
    print(args.output / 'index.html', flush=True)


if __name__ == '__main__':
    main()
