from dataclasses import MISSING, dataclass
from typing import Literal

from taser_training.BC.utils.dataset import ACTION_DIM, OBS_DIM


@dataclass
class GPTConfig:
    type: Literal["ACT", "history"] = MISSING
    seq_len: int = 0
    history: int = 0
    is_causal: bool = False
    action_chunk_size: int = 0

    obs_dim: int = OBS_DIM
    action_dim: int = ACTION_DIM
    n_layer: int = 3
    n_head: int = 3
    n_embd: int = 210
    dropout: float = 0.0
    bias: bool = True  # True: bias in Linears and LayerNorms, like GPT-2. False: a bit better and faster
    predict_eef_pos: bool = False

    def __post_init__(self):
        if self.type == "ACT":
            self.seq_len = 4
            self.history = 1
            self.is_causal = False
            self.action_chunk_size = 10
        elif self.type == "history":
            self.seq_len = 30
            self.history = 30
            self.is_causal = True
            self.action_chunk_size = 1
        else:
            raise ValueError(f"Unknown GPTConfig type: {self.type}")
