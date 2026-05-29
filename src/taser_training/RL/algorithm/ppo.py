import torch
import torch.nn.functional as F
from isaaclab.envs.manager_based_rl_env import ManagerBasedRLEnv
from torch.distributions import kl_divergence

from taser_training.RL.model.actor_critic import ActorCritic

from .train_cfg import TrainCfg


class PPO:
    def __init__(self, env: ManagerBasedRLEnv, cfg: TrainCfg):
        self.env = env
        self.cfg = cfg

        obs_dict_dims = {
            k: v.shape[1] for k, v in env.unwrapped.observation_space.items()
        }
        act_dim = env.unwrapped.action_space.shape[1]

        # Initialize policy network
        self.policy = ActorCritic(obs_dict_dims, act_dim).to(cfg.device)
        self.policy.train()

        self.optimizer = torch.optim.Adam(
            self.policy.parameters(), lr=cfg.learning_rate
        )

        self.buffers = dict()
        self.reset_buffers()

    def update(self, iter: int, final_val: torch.Tensor) -> dict[str, float]:
        self.buffers["val"].append(final_val)

        self.update_learning_rate(iter=iter)

        obs_dict_buf = {
            k: torch.stack([d[k] for d in self.buffers["obs_dict"]])
            for k in self.buffers["obs_dict"][0]
        }  # (T, N, obs_dim)
        act_buf = torch.stack(self.buffers["act"])  # (T, N, act_dim)
        rew_buf = torch.stack(self.buffers["rew"])  # (T, N)
        done_buf = torch.stack(self.buffers["done"])  # (T, N)
        mu_buf = torch.stack(self.buffers["mu"])  # (T, N, act_dim)
        std_buf = torch.stack(self.buffers["std"])  # (T, N, act_dim)
        val_buf = torch.stack(self.buffers["val"])  # (T+1, N)

        # Compute advantages and returns
        adv_buf, ret_buf = self.compute_gae(rew_buf, val_buf, done_buf)

        # Flatten buffers
        obs_dict_flat = {
            k: v.reshape(-1, v.shape[-1]) for k, v in obs_dict_buf.items()
        }  # (T*N, obs_dim)
        act_flat = act_buf.reshape(-1, act_buf.shape[-1])
        adv_flat = (adv_buf.reshape(-1) - adv_buf.mean()) / (adv_buf.std() + 1e-8)
        ret_flat = ret_buf.reshape(-1)
        mu_old_flat = mu_buf.reshape(-1, mu_buf.shape[-1])
        std_old_flat = std_buf.reshape(-1, std_buf.shape[-1])

        # Create old distribution
        old_action_dist = torch.distributions.Normal(mu_old_flat, std_old_flat)
        log_prob_old = old_action_dist.log_prob(act_flat).sum(-1)

        # Stats
        total_loss = 0
        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0
        total_kl = 0

        # PPO update
        for _ in range(self.cfg.num_epochs):
            action_dist, value = self.policy(obs_dict_flat, update_norm=False)
            log_prob = action_dist.log_prob(act_flat).sum(-1)
            entropy = action_dist.entropy().mean()

            kl = kl_divergence(old_action_dist, action_dist).mean()

            policy_ratio = torch.exp(log_prob - log_prob_old)
            full_loss = policy_ratio * adv_flat
            clipped_loss = (
                torch.clamp(
                    policy_ratio, 1.0 - self.cfg.clip_eps, 1.0 + self.cfg.clip_eps
                )
                * adv_flat
            )
            policy_loss = -torch.min(full_loss, clipped_loss).mean()
            value_loss = F.mse_loss(value, ret_flat)

            loss: torch.Tensor = (
                policy_loss
                + self.cfg.vf_coef * value_loss
                - self.cfg.ent_coef * entropy
            )

            total_loss += loss.item()
            total_policy_loss += policy_loss.item()
            total_value_loss += value_loss.item()
            total_entropy += entropy.item()
            total_kl += kl.item()

            if kl > 1.5 * self.cfg.target_kl:
                break

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        self.env.unwrapped.num_ppo_updates += 1
        num_epochs = _ + 1  # Actual number of epochs completed

        self.reset_buffers()

        return {
            "mean_reward": rew_buf.mean().item() / self.env.unwrapped.step_dt,
            "mean_return": ret_buf.mean().item() / self.env.unwrapped.step_dt,
            "total_loss": total_loss / num_epochs,
            "policy_loss": total_policy_loss / num_epochs,
            "value_loss": total_value_loss / num_epochs,
            "entropy": total_entropy / num_epochs,
            "kl_divergence": total_kl / num_epochs,
            "common_step_counter": self.env.unwrapped.common_step_counter,
            "learning_rate": self.optimizer.param_groups[0]["lr"],
        }

    def update_learning_rate(self, iter: int) -> None:
        """Update learning rate using exponential decay schedule."""
        if self.cfg.lr_decay_factor == 1.0:
            return
        lr = self.cfg.learning_rate * (self.cfg.lr_decay_factor**iter)
        for param_group in self.optimizer.param_groups:
            param_group["lr"] = lr

    def reset_buffers(self):
        self.buffers = {
            "obs_dict": [],
            "act": [],
            "rew": [],
            "done": [],
            "mu": [],
            "std": [],
            "val": [],
        }

    def update_buffers(
        self,
        obs_dict: dict[str, torch.Tensor],
        action: torch.Tensor,
        reward: torch.Tensor,
        done: torch.Tensor,
        action_dist: torch.distributions.Distribution,
        value: torch.Tensor,
    ):
        # TODO: Figure out why NaNs are showing up, potentially causing the fluctuations in rewards and value loss graphs
        obs_dict = {k: torch.nan_to_num(v, nan=0.0) for k, v in obs_dict.items()}
        action = torch.nan_to_num(action, nan=0.0)
        reward = torch.nan_to_num(reward, nan=0.0)
        done = torch.nan_to_num(done, nan=0.0)

        self.buffers["obs_dict"].append(obs_dict)
        self.buffers["act"].append(action)
        self.buffers["rew"].append(reward)
        self.buffers["done"].append(done)
        self.buffers["mu"].append(action_dist.loc)
        self.buffers["std"].append(action_dist.scale)
        self.buffers["val"].append(value)

    def compute_gae(
        self, rewards: torch.Tensor, values: torch.Tensor, dones: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Compute Generalized Advantage Estimation (GAE).

        Args:
            rewards: Tensor of shape (T, N) where T is time steps and N is number of envs
            values: Tensor of shape (T+1, N) including the final value estimate
            dones: Tensor of shape (T, N)
            gamma: Discount factor
            lam: GAE lambda parameter

        Returns:
            advantages: Tensor of shape (T, N)
            returns: Tensor of shape (T, N)
        """
        T = rewards.shape[0]
        advantages = torch.zeros_like(rewards)
        last_gae = 0

        for t in reversed(range(T)):
            next_value = values[t + 1]
            next_non_terminal = torch.logical_not(dones[t])

            delta = (
                rewards[t] + self.cfg.gamma * next_value * next_non_terminal - values[t]
            )
            last_gae = (
                delta
                + self.cfg.gamma * self.cfg.gae_lambda * next_non_terminal * last_gae
            )
            advantages[t] = last_gae

        returns = advantages + values[:-1]
        return advantages, returns
