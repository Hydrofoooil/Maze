import json
from typing import Any, Dict, Iterable, Tuple

import numpy as np


MODEL_VERSION = 1
JOINT_KEYS = ("b", "s", "e", "w", "h")
TORQUE_KEYS = ("torB", "torS", "torE", "torW", "torH")


def features(q_rad: Iterable[float]) -> np.ndarray:
    """Static torque-bias features from joint posture, q in radians."""
    q = np.asarray(list(q_rad), dtype=float)
    return np.concatenate([
        np.ones(1, dtype=float),
        np.sin(q),
        np.cos(q),
        np.sin(2.0 * q),
        np.cos(2.0 * q),
    ])


def state_to_q_tau(sample: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """Read a raw T=1051-like sample into q(rad) and torque(N*cm)."""
    q = np.array([
        float(sample.get("b", 0.0)),
        float(sample.get("s", 0.0)),
        float(sample.get("e", 0.0)),
        float(sample.get("w", 0.0)),
        float(sample.get("t", sample.get("h", 0.0))),
    ], dtype=float)
    tau = np.array([float(sample.get(k, 0.0)) for k in TORQUE_KEYS],
                   dtype=float)
    return q, tau


def fit(samples: Iterable[Dict[str, Any]], ridge: float = 1e-6) -> Dict[str, Any]:
    qs, taus = [], []
    for sample in samples:
        q, tau = state_to_q_tau(sample)
        qs.append(q)
        taus.append(tau)
    if len(qs) < 8:
        raise ValueError("need at least 8 calibration samples")

    X = np.vstack([features(q) for q in qs])
    Y = np.vstack(taus)
    reg = float(ridge) * np.eye(X.shape[1], dtype=float)
    coef = np.linalg.solve(X.T @ X + reg, X.T @ Y)
    pred = X @ coef
    rmse = np.sqrt(np.mean((pred - Y) ** 2, axis=0))

    return {
        "version": MODEL_VERSION,
        "joint_keys": list(JOINT_KEYS),
        "torque_keys": list(TORQUE_KEYS),
        "units": {
            "q": "rad",
            "torque": "N*cm",
        },
        "feature": "1,sin(q),cos(q),sin(2q),cos(2q)",
        "ridge": float(ridge),
        "n_samples": len(qs),
        "coef": coef.tolist(),
        "rmse_ncm": {k: float(rmse[i]) for i, k in enumerate(TORQUE_KEYS)},
    }


def predict(model: Dict[str, Any], q_rad: Iterable[float]) -> np.ndarray:
    coef = np.asarray(model["coef"], dtype=float)
    return features(q_rad) @ coef


def load(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        model = json.load(f)
    if int(model.get("version", -1)) != MODEL_VERSION:
        raise ValueError(f"unsupported force bias model version: "
                         f"{model.get('version')}")
    return model


def save(model: Dict[str, Any], path: str):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(model, f, ensure_ascii=False, indent=2)
