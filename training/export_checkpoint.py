"""export_checkpoint.py -- export a train_ppo.py checkpoint (.pt) to the deployable ONNX (normalizer baked in).

    python training/export_checkpoint.py training/runs/walk_v4_ft/last.pt [--out policy.onnx]

The result has the same contract as the pretrained policies (input "obs" [1, 61] -> "actions" [1, 14]); evaluate
it with  python scripts/motions.py all --walk-policy <onnx>.
"""
import argparse, sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_ppo import ActorCritic, export_onnx  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint"); ap.add_argument("--out")
    a = ap.parse_args(argv)
    ck = torch.load(a.checkpoint, weights_only=False)
    ac = ActorCritic(); ac.load_state_dict(ck["model"])
    out = Path(a.out or Path(a.checkpoint).with_suffix(".onnx"))
    export_onnx(ac, out, dict(producer="duck/training/export_checkpoint.py", checkpoint=Path(a.checkpoint).name,
                              iteration=ck["iter"], init_onnx=ck["args"].get("init_onnx") or "",
                              model=Path(ck["args"]["model"]).name, obs_dim=61, action_dim=14, control_hz=50))
    print(f"wrote {out} (iteration {ck['iter']})")


if __name__ == "__main__":
    main()
