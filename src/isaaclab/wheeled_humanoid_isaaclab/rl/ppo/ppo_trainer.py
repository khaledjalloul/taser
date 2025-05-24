import torch
import torch.nn.functional as F
from gymnasium import Env

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
        # Reset environment
        obs_dict, _ = self.env.reset()
        obs = torch.tensor(
            obs_dict["policy"], dtype=torch.float32, device=self.cfg.device)

        # Initialize buffers
        obs_buf = []
        act_buf = []
        logp_buf = []
        rew_buf = []
        val_buf = []
        done_buf = []

        # Collect experience
        for _ in range(self.cfg.num_steps):
            with torch.no_grad():
                dist, value = self.policy(obs)
                action = dist.sample()
                logp = dist.log_prob(action).sum(-1)

            # Step environment
            next_obs_dict, reward, terminated, truncated, _ = self.env.step(
                action)
            next_obs = torch.tensor(
                next_obs_dict["policy"], dtype=torch.float32, device=self.cfg.device)
            done = torch.logical_or(terminated, truncated)

            # Store transitions
            obs_buf.append(obs)
            act_buf.append(action)
            logp_buf.append(logp)
            rew_buf.append(torch.tensor(
                reward, dtype=torch.float32, device=self.cfg.device))
            val_buf.append(value)
            done_buf.append(torch.tensor(
                done, dtype=torch.float32, device=self.cfg.device))

            obs = next_obs

        # Convert buffers to tensors
        obs_buf = torch.stack(obs_buf)  # (T, N, obs_dim)
        act_buf = torch.stack(act_buf)  # (T, N, act_dim)
        logp_buf = torch.stack(logp_buf)  # (T, N)
        rew_buf = torch.stack(rew_buf)  # (T, N)
        val_buf = torch.stack(val_buf)  # (T, N)
        done_buf = torch.stack(done_buf)  # (T, N)

        # Compute final value for bootstrapping
        with torch.no_grad():
            _, final_val = self.policy(obs)
            val_buf = torch.cat(
                [val_buf, final_val.unsqueeze(0)], dim=0)  # (T+1, N)

        # Compute advantages and returns
        adv_buf, ret_buf = self.compute_gae(
            rew_buf,
            val_buf,
            done_buf
        )

        # Flatten buffers
        obs_flat = obs_buf.reshape(-1, obs_buf.shape[-1])
        act_flat = act_buf.reshape(-1, act_buf.shape[-1])
        logp_old_flat = logp_buf.reshape(-1)
        adv_flat = (adv_buf.reshape(-1) - adv_buf.mean()) / \
            (adv_buf.std() + 1e-8)
        ret_flat = ret_buf.reshape(-1)

        # PPO update
        total_loss = 0
        for _ in range(self.cfg.batch_epochs):
            dist, value = self.policy(obs_flat)
            logp = dist.log_prob(act_flat).sum(-1)
            entropy = dist.entropy().mean()

            ratio = torch.exp(logp - logp_old_flat)
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 1.0 - self.cfg.clip_eps,
                                1.0 + self.cfg.clip_eps) * adv_flat
            policy_loss = -torch.min(surr1, surr2).mean()
            value_loss = F.mse_loss(value, ret_flat)

            loss = policy_loss + self.cfg.vf_coef * value_loss - \
                self.cfg.ent_coef * entropy

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            update_info = {
                'policy_loss': policy_loss.item(),
                'value_loss': value_loss.item(),
                'entropy': entropy.item()
            }
            total_loss += sum(update_info.values())

        update_info['loss'] = total_loss / self.cfg.batch_epochs
        return update_info

    def save_model(self, path: str):
        torch.save(self.policy.state_dict(), path)

    def load_model(self, path: str):
        self.policy.load_state_dict(torch.load(
            path, map_location=self.cfg.device))

    def compute_gae(self, rewards, values, dones):
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
            next_non_terminal = 1.0 - dones[t]

            delta = rewards[t] + self.cfg.gamma * next_value * \
                next_non_terminal - values[t]
            last_gae = delta + self.cfg.gamma * \
                self.cfg.gae_lambda * next_non_terminal * last_gae
            advantages[t] = last_gae

        returns = advantages + values[:-1]
        return advantages, returns
