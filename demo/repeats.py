"""Fixed repeated-scan endpoint/normal consistency; references stay unchanged."""
import numpy as np


class RepeatedScans:
    def __init__(self, path):
        with np.load(path, allow_pickle=False) as data:
            self.data = {name: data[name].copy() for name in data.files}

    @staticmethod
    def summarize(groups):
        values = np.concatenate(groups)
        weights = np.concatenate([np.full(len(v), 1 / len(v) / len(groups)) for v in groups])
        order = np.argsort(values)
        index = min(len(values) - 1, np.searchsorted(np.cumsum(weights[order]), .95))
        return dict(queries=len(values), directions=len(groups),
                    mean=float(weights @ values), p95=float(values[order[index]]))

    def score(self, trajectory):
        d = self.data
        result = {}
        for region in ['whole', 'central']:
            planes, points = [], []
            for k, (i, j) in enumerate(d['frames']):
                begin, end = d['offsets'][k:k + 2]
                mask = np.ones(end - begin, dtype=bool) if region == 'whole' else d['central'][begin:end]
                if not mask.any():
                    continue
                yaw = trajectory[i, 2] - trajectory[j, 2]
                c, s = np.cos(yaw), np.sin(yaw)
                cj, sj = np.cos(trajectory[j, 2]), np.sin(trajectory[j, 2])
                translation = (trajectory[i, :2] - trajectory[j, :2]) @ np.array([[cj, -sj], [sj, cj]])
                delta = d['p'][begin:end][mask] @ np.array([[c, s], [-s, c]]) + translation - d['q'][begin:end][mask]
                planes.append(np.abs(np.einsum('ij,ij->i', d['n'][begin:end][mask], delta)))
                points.append(np.linalg.norm(delta, axis=1))
            result[region] = dict(plane=self.summarize(planes), point=self.summarize(points))
        return result
