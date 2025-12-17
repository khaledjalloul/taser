import torch
from torch import nn

from .config import TransformerCfg
from .utils import Block, LayerNorm


class GPT(nn.Module):
    def __init__(self, config=TransformerCfg(type="GPT")):
        super().__init__()
        self.config = config

        self.transformer = nn.ModuleDict(
            dict(
                token_embed=nn.Linear(self.config.obs_dim, self.config.n_embd),
                pos_embed=nn.Embedding(self.config.seq_len, self.config.n_embd),
                drop=nn.Dropout(self.config.dropout),
                h=nn.ModuleList(
                    [Block(self.config) for _ in range(self.config.n_layer)]
                ),
                ln_f=LayerNorm(self.config.n_embd, bias=self.config.bias),
            )
        )

        self.action_head = nn.Linear(
            self.config.n_embd, self.config.action_dim * self.config.action_chunk_size
        )

        if self.config.predict_eef_pos:
            self.eef_pos_head = nn.Linear(
                self.config.n_embd, 6 * self.config.action_chunk_size
            )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        B, T, _ = x.shape

        # Forward pass through transformer
        tok_emb = self.transformer.token_embed(x)
        pos_emb = self.transformer.pos_embed(
            torch.arange(0, T, dtype=torch.long, device=x.device)
        )
        x = self.transformer.drop(tok_emb + pos_emb)

        att_weights_list = []
        for block in self.transformer.h:
            x, att_weights = block(x)
            if att_weights is not None:
                att_weights_list.append(att_weights)
        x = self.transformer.ln_f(x)

        # Final action prediction head
        actions = self.action_head(x)
        actions = actions.view(
            B, T, self.config.action_chunk_size, self.config.action_dim
        )
        output = {"actions": actions}

        # Optional end-effector position prediction head
        if self.config.predict_eef_pos:
            eef_pos = self.eef_pos_head(x)
            eef_pos = eef_pos.view(B, T, self.config.action_chunk_size, 6)
            output["eef_pos"] = eef_pos

        return output


if __name__ == "__main__":
    from taser.common.logger import logger

    num_time_steps = 25

    # Simple test
    model = GPT()

    x = torch.randn(
        10, num_time_steps, 24
    )  # (batch_size, num_time_steps, obs_dim) -> (B, T, chunk_size, 5)
    out = model(x)
    logger.info(out["actions"].shape)  # Expected: (batch_size, num_time_steps, act_dim)
