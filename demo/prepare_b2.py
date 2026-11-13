#!/usr/bin/env python3
"""Extract b2 first-echo scans at the exact prepared trajectory timestamps."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
import yaml


def prepare(bag, data, output):
    output.mkdir(parents=True, exist_ok=False)
    metadata = json.loads((data / 'preparation.json').read_text())
    expected = np.loadtxt(data / 'timestamps_ns.txt', dtype=np.int64)
    if len(expected) != metadata['frames']:
        raise ValueError('Unexpected prepared trajectory length')
    count = 0
    ranges = output / 'Range.txt.gz'
    with AnyReader([bag]) as reader, gzip.open(ranges, 'wt', compresslevel=1) as stream:
        selected = [c for c in reader.connections
                    if c.topic == metadata['topic'] and c.msgtype.endswith('/MultiEchoLaserScan')]
        if len(selected) != 1:
            raise ValueError('Expected one b2 horizontal MultiEchoLaserScan connection')
        for connection, _, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, connection.msgtype)
            timestamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if count >= len(expected) or timestamp != expected[count]:
                raise ValueError('Scan timestamps differ from the supplied initialization')
            if (len(msg.ranges) != metadata['beams'] or msg.angle_min != metadata['angle_min']
                    or msg.angle_max != metadata['angle_max']):
                raise ValueError('Unexpected b2 beam geometry')
            values = np.asarray([echo.echoes[0] if len(echo.echoes) else np.inf for echo in msg.ranges])
            np.savetxt(stream, values[None, :], fmt='%.17g')
            count += 1
            if count % 10000 == 0:
                print(f'Extracted {count:,} / {len(expected):,} scans', flush=True)
    if count != metadata['frames']:
        raise ValueError('Incomplete b2 scan stream')
    config = yaml.safe_load((data / 'demo.yaml').read_text())
    config['input'].update(ranges=str(ranges.resolve()), poses=str((data / 'Pose.txt').resolve()))
    config['submaps']['anchor_poses'] = str((data / 'InitialSubmapPoses.txt').resolve())
    path = output / 'demo.yaml'
    path.write_text(yaml.safe_dump(config))
    (output / 'input_receipt.json').write_text(json.dumps(dict(
        scans=count, beams=metadata['beams'], topic=metadata['topic'],
        all_header_times_exact=True, echo='first', text_precision='17 significant digits'), indent=2) + '\n')
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.bag.resolve(strict=True), args.data.resolve(strict=True), args.output.resolve()))
