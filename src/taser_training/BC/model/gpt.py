import math

import torch
import torch.nn as nn
from torch.nn import functional as F

from taser_training.BC.model.gpt_config import GPTConfig
from taser_training.BC.utils.dataset import GPTEpisode


class LayerNorm(nn.Module):
    """LayerNorm but with an optional bias. PyTorch doesn't support simply bias=False"""

    def __init__(self, ndim, bias):
        super().__init__()
        self.weight = nn.Parameter(torch.ones(ndim))
        self.bias = nn.Parameter(torch.zeros(ndim)) if bias else None

    def forward(self, input):
        return F.layer_norm(input, self.weight.shape, self.weight, self.bias, 1e-5)


class SelfAttention(nn.Module):
    def __init__(self, config: GPTConfig):
        super().__init__()
        assert config.n_embd % config.n_head == 0
        # key, query, value projections for all heads, but in a batch
        self.c_attn = nn.Linear(config.n_embd, 3 * config.n_embd, bias=config.bias)
        # output projection
        self.c_proj = nn.Linear(config.n_embd, config.n_embd, bias=config.bias)
        # regularization
        self.attn_dropout = nn.Dropout(config.dropout)
        self.resid_dropout = nn.Dropout(config.dropout)
        self.seq_len = config.seq_len
        self.is_causal = config.is_causal
        self.n_head = config.n_head
        self.n_embd = config.n_embd
        self.dropout = config.dropout
        # flash attention make GPU go brrrrr but support is only in PyTorch >= 2.0
        self.flash = hasattr(F, "scaled_dot_product_attention")

    def forward(
        self, x: torch.Tensor, attn_mask: torch.Tensor | None = None
    ) -> torch.Tensor:
        B, T, C = (
            x.size()
        )  # batch size, sequence length, embedding dimensionality (n_embd)

        # calculate query, key, values for all heads in batch and move head forward to be the batch dim
        q, k, v = self.c_attn(x).split(self.n_embd, dim=2)
        k = k.view(B, T, self.n_head, C // self.n_head).transpose(
            1, 2
        )  # (B, nh, T, hs)
        q = q.view(B, T, self.n_head, C // self.n_head).transpose(
            1, 2
        )  # (B, nh, T, hs)
        v = v.view(B, T, self.n_head, C // self.n_head).transpose(
            1, 2
        )  # (B, nh, T, hs)

        # causal self-attention; Self-attend: (B, nh, T, hs) x (B, nh, hs, T) -> (B, nh, T, T)
        if self.flash and (attn_mask is None or not self.is_causal):
            # efficient attention using Flash Attention CUDA kernels
            y = F.scaled_dot_product_attention(
                q,
                k,
                v,
                attn_mask=attn_mask,
                dropout_p=self.dropout if self.training else 0,
                is_causal=self.is_causal,
            )
        else:
            # manual implementation of attention
            att = (q @ k.transpose(-2, -1)) * (1.0 / math.sqrt(k.size(-1)))
            if self.is_causal:
                causal_mask = torch.tril(
                    torch.ones(self.seq_len, self.seq_len, device=x.device)
                ).view(1, 1, self.seq_len, self.seq_len)[:, :, :T, :T]
                att = att.masked_fill(causal_mask == 0, -1e9)
            if attn_mask is not None:
                att = att.masked_fill(attn_mask, -1e9)
            att = F.softmax(att, dim=-1)
            att = self.attn_dropout(att)
            y = att @ v  # (B, nh, T, T) x (B, nh, T, hs) -> (B, nh, T, hs)
        y = (
            y.transpose(1, 2).contiguous().view(B, T, C)
        )  # re-assemble all head outputs side by side

        # output projection
        y = self.resid_dropout(self.c_proj(y))
        return y


class CrossAttention(nn.Module):
    def __init__(self, config: GPTConfig):
        super().__init__()
        assert config.n_embd % config.n_head == 0
        self.n_head = config.n_head
        self.head_dim = config.n_embd // config.n_head

        # separate projections so Q comes from tokens, K/V from context
        self.w_q = nn.Linear(config.n_embd, config.n_embd)
        self.w_k = nn.Linear(config.n_embd, config.n_embd)
        self.w_v = nn.Linear(config.n_embd, config.n_embd)

        self.proj = nn.Linear(config.n_embd, config.n_embd)
        self.attn_drop = nn.Dropout(config.dropout)
        self.resid_drop = nn.Dropout(config.dropout)

    def forward(
        self, x: torch.Tensor, ctx: torch.Tensor, ctx_mask: torch.Tensor | None = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        x:   (B, Tq, C)  -- queries (decoder states)
        ctx: (B, Tk, C)  -- keys/values (encoder/context)
        ctx_mask: (B, Tk) boolean, True for VALID tokens (padding=False). Optional.
        """
        B, Tq, C = x.shape
        Tk = ctx.shape[1]

        q = (
            self.w_q(x).view(B, Tq, self.n_head, self.head_dim).transpose(1, 2)
        )  # (B,H,Tq,Dh)
        k = (
            self.w_k(ctx).view(B, Tk, self.n_head, self.head_dim).transpose(1, 2)
        )  # (B,H,Tk,Dh)
        v = (
            self.w_v(ctx).view(B, Tk, self.n_head, self.head_dim).transpose(1, 2)
        )  # (B,H,Tk,Dh)

        # Manual implementation to get attention weights
        att = (q @ k.transpose(-2, -1)) / (self.head_dim**0.5)  # (B,H,Tq,Tk)
        if ctx_mask is not None:
            # mask invalid keys with -inf
            att = att.masked_fill((~ctx_mask[:, None, None, :]), -1e9)
        att = F.softmax(att, dim=-1)
        att_weights = att
        att = self.attn_drop(att)
        y = att @ v

        y = y.transpose(1, 2).contiguous().view(B, Tq, C)  # (B,Tq,C)
        return self.resid_drop(self.proj(y)), att_weights


class MLP(nn.Module):
    def __init__(self, config: GPTConfig):
        super().__init__()
        self.c_fc = nn.Linear(config.n_embd, 4 * config.n_embd, bias=config.bias)
        self.gelu = nn.GELU()
        self.c_proj = nn.Linear(4 * config.n_embd, config.n_embd, bias=config.bias)
        self.dropout = nn.Dropout(config.dropout)

    def forward(self, x):
        x = self.c_fc(x)
        x = self.gelu(x)
        x = self.c_proj(x)
        x = self.dropout(x)
        return x


class Block(nn.Module):
    def __init__(self, config: GPTConfig):
        super().__init__()
        self.ln1 = LayerNorm(config.n_embd, bias=config.bias)
        self.self_attn = SelfAttention(config)
        self.ln2 = LayerNorm(config.n_embd, bias=config.bias)
        self.cross_attn = CrossAttention(config)
        self.ln3 = LayerNorm(config.n_embd, bias=config.bias)
        self.mlp = MLP(config)

    def forward(
        self,
        x,
        attn_mask: torch.Tensor | None = None,
        ctx: torch.Tensor | None = None,
        ctx_mask: torch.Tensor | None = None,
    ):
        x = x + self.self_attn(self.ln1(x), attn_mask=attn_mask)
        att_weights = None
        if ctx is not None:
            x_cross, att_weights = self.cross_attn(self.ln2(x), ctx, ctx_mask)
            x = x + x_cross
        x = x + self.mlp(self.ln3(x))
        return x, att_weights


class GPT_ACT(nn.Module):
    def __init__(self, config: GPTConfig = GPTConfig(type="ACT")):
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

        # --- Heads ---
        # x is (B, chunk_size, n_embd)
        actions = self.action_head(x)  # (B, chunk_size, action_dim)
        output = {"actions": actions}

        if self.config.predict_eef_pos:
            eef_pos = self.eef_pos_head(x)  # (B, chunk_size, 3)
            output["eef_pos"] = eef_pos

        return output


class GPT_History(nn.Module):
    def __init__(self, config=GPTConfig(type="history")):
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
        if self.config.action_chunk_size > 1:
            actions = actions.view(
                B, T, self.config.action_chunk_size, self.config.action_dim
            )
        output = {"actions": actions}

        # Optional end-effector position prediction head
        if self.config.predict_eef_pos:
            eef_pos = self.eef_pos_head(x)
            if self.config.action_chunk_size > 1:
                eef_pos = eef_pos.view(B, T, self.config.action_chunk_size, 6)
            output["eef_pos"] = eef_pos

        return output


if __name__ == "__main__":
    from taser.common.logger import logger

    num_time_steps = 25

    # Simple test
    model = GPT_History()

    x = torch.randn(
        10, num_time_steps, 24
    )  # (batch_size, num_time_steps, obs_dim) -> (B, T, chunk_size, 5)
    out = model(x)
    logger.info(out["actions"].shape)  # Expected: (batch_size, num_time_steps, act_dim)
