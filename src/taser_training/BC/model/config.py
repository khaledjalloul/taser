from dataclasses import MISSING, dataclass
from typing import Literal

from taser_training.BC.utils.dataset import ACTION_DIM, OBS_DIM


@dataclass
class TransformerCfg:
    type: Literal["ACT", "GPT"] = MISSING
    seq_len: int = 0
    history: int = 0
    is_causal: bool = False
    action_chunk_size: int = 0

    obs_dim: int = OBS_DIM
    action_dim: int = ACTION_DIM
    n_layer: int = 4
    n_head: int = 4
    n_embd: int = 512
    dropout: float = 0.0
    bias: bool = True  # True: bias in Linears and LayerNorms, like GPT-2. False: a bit better and faster
    predict_eef_pos: bool = True

    def __post_init__(self):
        if self.type == "ACT":
            self.seq_len = 4
            self.history = 1
            self.is_causal = False
            self.action_chunk_size = 30
        elif self.type == "GPT":
            self.seq_len = 20
            self.history = 20
            self.is_causal = True
            self.action_chunk_size = 30
        else:
            raise ValueError(f"Unknown model type: {self.type}")
