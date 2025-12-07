from dataclasses import dataclass

from taser_training.BC.utils.dataset import ACTION_DIM, OBS_DIM


@dataclass
class GPTConfig:
    obs_dim: int = OBS_DIM
    action_dim: int = ACTION_DIM
    seq_len: int = 1
    is_causal: bool = False
    n_layer: int = 3
    n_head: int = 3
    n_embd: int = 210
    dropout: float = 0.0
    bias: bool = True  # True: bias in Linears and LayerNorms, like GPT-2. False: a bit better and faster
    action_chunk_size: int = 50
    predict_eef_pos: bool = True
