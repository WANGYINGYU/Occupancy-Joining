"""Evaluate the Intel demo against independently matched relative laser poses."""
from pathlib import Path
import numpy as np


class Relations:
    def __init__(self, timestamps, reference):
        times = Path(timestamps).read_text().splitlines()
        lookup = {time: index for index, time in enumerate(times)}
        self.reference = np.loadtxt(reference, ndmin=2)
        if len(lookup) != len(times) or self.reference.shape[1] != 8 or not np.isfinite(self.reference).all():
            raise ValueError('Invalid relative-pose reference or duplicate input timestamps')
        self.indices = np.asarray([[lookup[f'{value:.6f}'] for value in row[:2]]
                                   for row in self.reference], dtype=int)
        self.frames = len(times)

    def errors(self, poses):
        poses = np.asarray(poses)
        if poses.shape != (self.frames, 3) or not np.isfinite(poses).all():
            raise ValueError('Evaluation trajectory does not match the source scans')
        first, second = poses[self.indices[:, 0]], poses[self.indices[:, 1]]
        delta = second[:, :2] - first[:, :2]
        c, s = np.cos(first[:, 2]), np.sin(first[:, 2])
        relative = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                    -s * delta[:, 0] + c * delta[:, 1]))
        translation = np.linalg.norm(relative - self.reference[:, 2:4], axis=1)
        yaw = second[:, 2] - first[:, 2] - self.reference[:, 7]
        rotation = np.abs(np.arctan2(np.sin(yaw), np.cos(yaw))) * 180 / np.pi
        return translation, rotation

    def score(self, poses):
        def summary(values):
            return dict(mean=float(values.mean()), median=float(np.median(values)),
                        rmse=float(np.sqrt(np.mean(values ** 2))),
                        p95=float(np.quantile(values, .95)), maximum=float(values.max()))
        translation, rotation = self.errors(poses)
        return dict(pairs=len(translation), translation_m=summary(translation),
                    rotation_deg=summary(rotation))

    def history(self, inputs, items, anchors):
        owner = np.full(self.frames, -1, dtype=int)
        source = np.zeros((len(items), 3))
        for index, item in enumerate(items):
            rows = np.arange(item['begin'], item['end'])
            owner[rows[owner[rows] < 0]] = index
            source[index] = inputs[item['anchor_index']]
        if (owner < 0).any():
            raise ValueError('Submap ownership does not cover all reference frames')
        original = source[owner]
        delta = inputs[:, :2] - original[:, :2]
        c, s = np.cos(original[:, 2]), np.sin(original[:, 2])
        local = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                -s * delta[:, 0] + c * delta[:, 1],
                                inputs[:, 2] - original[:, 2]))
        result = []
        for state in anchors:
            frame = state[owner]
            c, s = np.cos(frame[:, 2]), np.sin(frame[:, 2])
            poses = np.column_stack((c * local[:, 0] - s * local[:, 1] + frame[:, 0],
                                     s * local[:, 0] + c * local[:, 1] + frame[:, 1],
                                     local[:, 2] + frame[:, 2]))
            result.append(self.score(poses))
        return result
