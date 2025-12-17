import torch
from torch import nn

from taser_training.BC.utils.dataset import GPTEpisode

from .config import TransformerCfg
from .utils import Block, LayerNorm


class ACT(nn.Module):
    def __init__(self, config: TransformerCfg = TransformerCfg(type="ACT")):
        super().__init__()
        self.config = config
        self.common_token_dim = 64

        # Encoder
        self.encoder = nn.ModuleDict(
            dict(
                token_embed=nn.Linear(self.common_token_dim, self.config.n_embd),
                pos_embed=nn.Embedding(self.config.seq_len, self.config.n_embd),
                drop=nn.Dropout(self.config.dropout),
                h=nn.ModuleList(
                    [Block(self.config) for _ in range(self.config.n_layer)]
                ),
                ln_f=LayerNorm(self.config.n_embd, bias=self.config.bias),
            )
        )

        # Decoder (ACT style)
        self.decoder = nn.ModuleDict(
            dict(
                drop=nn.Dropout(self.config.dropout),
                h=nn.ModuleList(
                    [Block(self.config) for _ in range(self.config.n_layer)]
                ),
                ln_f=LayerNorm(self.config.n_embd, bias=self.config.bias),
            )
        )

        self.x_encoders = nn.ModuleList(
            [
                nn.Linear(sl.stop - sl.start, self.common_token_dim)
                for sl in GPTEpisode.indices.values()
            ]
        )

        # Action Queries
        self.action_chunk_size = self.config.action_chunk_size
        self.action_queries = nn.Parameter(
            torch.randn(1, self.action_chunk_size, self.config.n_embd)
        )
        self.query_pos_embed = nn.Parameter(
            torch.randn(1, self.action_chunk_size, self.config.n_embd)
        )

        # Heads (applied per query token)
        self.action_head = nn.Linear(self.config.n_embd, self.config.action_dim)

        if self.config.predict_eef_pos:
            self.eef_pos_head = nn.Linear(self.config.n_embd, 6)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        B, T, _ = x.shape

        # --- Encoder ---
        x_tokens = torch.zeros(
            (B, self.config.seq_len, self.common_token_dim), device=x.device
        )
        for idxs_i, idxs in enumerate(GPTEpisode.indices.values()):
            x_raw = x[:, 0, idxs]
            x_tokens[:, idxs_i] = self.x_encoders[idxs_i](x_raw)

        # Positional embeddings
        pos_ids = torch.arange(
            0, self.config.seq_len, dtype=torch.long, device=x.device
        )

        tok_emb = self.encoder.token_embed(x_tokens)
        pos_emb = self.encoder.pos_embed(pos_ids)
        memory = self.encoder.drop(tok_emb + pos_emb)

        for block in self.encoder.h:
            memory, _ = block(memory)  # ctx is None
        memory = self.encoder.ln_f(memory)

        # --- Decoder ---
        # Queries
        queries = self.action_queries.expand(B, -1, -1)  # (B, chunk_size, n_embd)
        queries = queries + self.query_pos_embed  # Add pos embed to queries

        x = self.decoder.drop(queries)
        for block in self.decoder.h:
            x, _ = block(x, ctx=memory)  # Cross-attend to memory
        x = self.decoder.ln_f(x)  # (B, chunk_size, n_embd)

        output = {}

        # --- Heads ---
        # x is (B, chunk_size, n_embd)
        actions = self.action_head(x)  # (B, chunk_size, action_dim)
        output["actions"] = actions.unsqueeze(1)  # (B, 1, chunk_size, action_dim)

        if self.config.predict_eef_pos:
            eef_pos = self.eef_pos_head(x)  # (B, chunk_size, 3)
            output["eef_pos"] = eef_pos.unsqueeze(1)  # (B, 1, chunk_size, 3)

        return output
