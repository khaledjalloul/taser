import torch
import torch.nn.functional as F
from gymnasium import Env
from torch.distributions import kl_divergence

from .actor_critic import ActorCritic
from .ppo_trainer_cfg import PPOTrainerCfg


class PPOTrainer:
    def __init__(self, env: Env, cfg: PPOTrainerCfg):
        self.env = env
        self.cfg = cfg

        # Initialize policy network
        obs_dim = env.observation_space["policy"].shape[1]
        act_dim = env.action_space.shape[1]
        self.policy = ActorCritic(obs_dim, act_dim).to(cfg.device)
        self.optimizer = torch.optim.Adam(
            self.policy.parameters(), lr=cfg.learning_rate)

    def train_step(self):
        # Rollout
        obs, act, adv, ret, mu_old, std_old = self.rollout()

        # Create old distribution
        old_action_dist = torch.distributions.Normal(mu_old, std_old)
        log_prob_old = old_action_dist.log_prob(act).sum(-1)

        # Stats
        total_loss = 0
        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0
        total_kl = 0

        # PPO update
        for _ in range(self.cfg.batch_epochs):
            action_dist, value = self.policy(obs)
            log_prob = action_dist.log_prob(act).sum(-1)
            entropy = action_dist.entropy().mean()

            # Calculate KL divergence using stored old distribution
            kl = kl_divergence(old_action_dist, action_dist).mean()

            policy_ratio = torch.exp(log_prob - log_prob_old)
            full_loss = policy_ratio * adv
            clipped_loss = torch.clamp(policy_ratio, 1.0 - self.cfg.clip_eps,
                                       1.0 + self.cfg.clip_eps) * adv
            policy_loss = -torch.min(full_loss, clipped_loss).mean()
            value_loss = F.mse_loss(value, ret)

            loss: torch.Tensor = policy_loss + self.cfg.vf_coef * \
                value_loss - self.cfg.ent_coef * entropy

            # Optional early stopping based on KL
            if kl > 1.5 * self.cfg.target_kl:
                break

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()
            total_policy_loss += policy_loss.item()
            total_value_loss += value_loss.item()
            total_entropy += entropy.item()
            total_kl += kl.item()

        num_epochs = _ + 1  # Actual number of epochs completed
        return {
            'loss': total_loss / num_epochs,
            'policy_loss': total_policy_loss / num_epochs,
            'value_loss': total_value_loss / num_epochs,
            'entropy': total_entropy / num_epochs,
            'kl': total_kl / num_epochs,
            'num_epochs': num_epochs
        }

    def rollout(self):
        # Reset environment
        obs_dict, _ = self.env.reset()
        obs = obs_dict["policy"]

        # Initialize buffers
        obs_buf = []
        act_buf = []
        rew_buf = []
        val_buf = []
        done_buf = []
        mu_buf = []
        std_buf = []

        # Collect experience
        for _ in range(self.cfg.num_steps):
            with torch.no_grad():
                action_dist, value = self.policy(obs)
                action = action_dist.sample()

            # Step environment
            next_obs_dict, reward, terminated, truncated, _ = \
                self.env.step(action)
            next_obs = next_obs_dict["policy"]
            done = torch.logical_or(terminated, truncated)

            # Store transitions
            obs_buf.append(obs)
            act_buf.append(action)
            rew_buf.append(reward)
            val_buf.append(value)
            done_buf.append(done)
            mu_buf.append(action_dist.loc)
            std_buf.append(action_dist.scale)

            obs = next_obs

        # Compute final value for bootstrapping
        with torch.no_grad():
            _, final_val = self.policy(obs)
            val_buf.append(final_val)

        # Convert buffers to tensors
        obs_buf = torch.stack(obs_buf)  # (T, N, obs_dim)
        act_buf = torch.stack(act_buf)  # (T, N, act_dim)
        rew_buf = torch.stack(rew_buf)  # (T, N)
        val_buf = torch.stack(val_buf)  # (T+1, N)
        done_buf = torch.stack(done_buf)  # (T, N)
        mu_buf = torch.stack(mu_buf)  # (T, N, act_dim)
        std_buf = torch.stack(std_buf)  # (T, N, act_dim)

        # Compute advantages and returns
        adv_buf, ret_buf = self.compute_gae(
            rew_buf,
            val_buf,
            done_buf
        )

        # Flatten buffers
        obs_flat = obs_buf.reshape(-1, obs_buf.shape[-1])
        act_flat = act_buf.reshape(-1, act_buf.shape[-1])
        adv_flat = (adv_buf.reshape(-1) - adv_buf.mean()) / \
            (adv_buf.std() + 1e-8)
        ret_flat = ret_buf.reshape(-1)
        mu_old_flat = mu_buf.reshape(-1, mu_buf.shape[-1])
        std_old_flat = std_buf.reshape(-1, std_buf.shape[-1])

        return obs_flat, act_flat, adv_flat, ret_flat, mu_old_flat, std_old_flat

    def compute_gae(self, rewards: torch.Tensor, values: torch.Tensor, dones: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
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

            delta = rewards[t] + self.cfg.gamma * next_value * \
                next_non_terminal - values[t]
            last_gae = delta + self.cfg.gamma * \
                self.cfg.gae_lambda * next_non_terminal * last_gae
            advantages[t] = last_gae

        returns = advantages + values[:-1]
        return advantages, returns

    def save_model(self, path: str):
        torch.save(self.policy.state_dict(), path)

    def load_model(self, path: str):
        self.policy.load_state_dict(
            torch.load(
                path,
                map_location=self.cfg.device,
                weights_only=True
            )
        )
