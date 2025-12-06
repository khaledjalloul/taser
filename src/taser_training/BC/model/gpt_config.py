from dataclasses import dataclass


@dataclass
class GPTConfig:
    obs_dim: int = 24  # 6 jp, 6 jv, 6 eef, 6 target
    action_dim: int = 6  # 6 joint velocities
    seq_len: int = 1
    is_causal: bool = False
    n_layer: int = 3
    n_head: int = 3
    n_embd: int = 210
    dropout: float = 0.0
    bias: bool = True  # True: bias in Linears and LayerNorms, like GPT-2. False: a bit better and faster
    action_chunk_size: int = 10
    predict_eef_pos: bool = True
