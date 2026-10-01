# KIRL

Code for *Kinematics-Informed Reinforcement Learning for Unified Trajectory Optimization in CNC Interpolation* (IEEE Transactions on Industrial Informatics, accepted).

KIRL unifies corner smoothing and feedrate scheduling as a segment-level Markov decision process. At each corner the policy chooses the junction kinematic state (position, velocity, acceleration); a minimum-jerk quintic primitive then connects consecutive junctions in the shortest duration that satisfies the tolerance and kinematic limits.

## Release status

This is a partial release. It currently contains the quintic motion primitives and the eight benchmark toolpaths. The trained universal policy, the inference-time planner and the scripts reproducing the paper's experiments will be released after the paper is formally published.

| Component | Status |
|---|---|
| Quintic motion primitives (`quintic.py`) | released |
| Benchmark toolpaths (`data/`) | released |
| Trained universal policy and planner (Algorithm 1) | after publication |
| Reproduction of Table III and Sec. IV-D timings | after publication |

## Install

```bash
conda create -n kirl python=3.10 && conda activate kirl
pip install -r requirements.txt
```

## Data

Eight planar benchmark toolpaths designed by the authors, stored as `data/<name>/data.txt` (tab-separated XY, mm). The universal policy was trained on six of them; **Golden Fish** and **Shark** are held out and never seen during training.

```python
import numpy as np
waypoints = np.loadtxt("data/shark/data.txt", delimiter="\t")   # (N, 2), mm
```

## Quintic motion primitives

Each trajectory segment is the minimum-jerk motion between two boundary kinematic states (Theorem 1): a fifth-degree polynomial per axis whose coefficients follow in closed form from the position, velocity and acceleration at both endpoints and the duration `T`.

```python
from quintic import KinematicState, boundary_conditions, coefficients, sample

x0 = KinematicState([0.0, 0.0], [5.0, 0.0], [0.0, 0.0])   # position, velocity, acceleration
x1 = KinematicState([2.0, 0.1], [5.0, 1.0], [0.0, 0.0])
C = coefficients(boundary_conditions(x0, x1), T=0.4)       # (6, 2) coefficient matrix
pos, vel, acc, jerk = sample(C, T=0.4, dt=0.0005)          # dense profiles along the segment
```

In KIRL the policy chooses `x1` at every corner, and the planner searches for the smallest feasible `T`.

## Results

Zero-shot generalization of the universal policy (paper Sec. IV-C, Table III). Kinematic limits (Table I): deviation ≤ 0.5 mm, velocity ≤ 10 mm/s, acceleration ≤ 100 mm/s², jerk ≤ 10000 mm/s³. `*` = held-out toolpath.

| Toolpath | Segments | T_m (s) | max d (mm) | max v | max a | max j | Fallbacks |
|---|---|---|---|---|---|---|---|
| Digit 3 | 326 | 47.985 | 0.118 | 9.40 | 100.0 | 8655 | 0 |
| Mermaid | 604 | 89.574 | 0.129 | 9.40 | 100.0 | 9864 | 0 |
| Unicorn | 483 | 72.557 | 0.154 | 9.38 | 100.0 | 10000 | 0 |
| Simple Wave | 240 | 35.171 | 0.017 | 10.00 | 100.0 | 9967 | 0 |
| Dolphin | 267 | 39.306 | 0.076 | 9.39 | 100.0 | 10000 | 0 |
| Manta Ray | 330 | 48.207 | 0.097 | 9.40 | 100.0 | 10000 | 0 |
| Golden Fish* | 382 | 56.323 | 0.126 | 9.41 | 100.0 | 8192 | 0 |
| Shark* | 244 | 36.284 | 0.120 | 9.40 | 100.0 | 8451 | 0 |

Full planning takes 0.08–0.10 ms per junction and 21–53 ms per toolpath on a single CPU core (Sec. IV-D).

## Citation

```bibtex
@article{zhang2026kirl,
  title   = {Kinematics-Informed Reinforcement Learning for Unified Trajectory Optimization in CNC Interpolation},
  author  = {Zhang, Jin and Zhao, Mingyang and Liu, Bing and Yan, Dong-Ming and Jiang, Xin},
  journal = {IEEE Transactions on Industrial Informatics},
  year    = {2026},
  note    = {Accepted}
}
```

## License

MIT, see [LICENSE](LICENSE).
