from dataclasses import dataclass

from taser_training.BC.utils.dataset import ACTION_DIM, OBS_DIM


@dataclass
class GPTCfg:
    obs_dim: int = OBS_DIM
    action_dim: int = ACTION_DIM
    seq_len: int = 20
    history: int = 20
    is_causal: bool = True
    action_chunk_size: int = 30
    n_layer: int = 4
    n_head: int = 4
    n_embd: int = 512
    dropout: float = 0.0
    bias: bool = True  # True: bias in Linears and LayerNorms, like GPT-2. False: a bit better and faster
    predict_eef_pos: bool = True


@dataclass
class MLPCfg:
    obs_dim: int = OBS_DIM
    action_dim: int = ACTION_DIM
    hidden_layers: list[int] = (128, 128)
    history: int = 1
    action_chunk_size: int = 1
    predict_eef_pos: bool = False
