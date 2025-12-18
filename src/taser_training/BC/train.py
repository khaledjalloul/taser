import argparse

parser = argparse.ArgumentParser(description="Train a GPT model for behavior cloning.")

parser.add_argument(
    "--data_path",
    type=str,
    required=True,
    help="Path to the dataset H5 file.",
)
parser.add_argument(
    "--model_type",
    type=str,
    required=True,
    choices=["ACT", "GPT"],
    help="Type of the transformer model to train (ACT or GPT).",
)
parser.add_argument("--resume", type=str, help="Path to checkpoint to resume from.")

args = parser.parse_args()

#############################################################

from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
from tqdm import tqdm

from taser.common.logger import logger
from taser_training.BC.model import ACT, GPT, TransformerCfg
from taser_training.BC.utils.dataset import GPTDataset
from taser_training.wandb_logger import WandbLogger


@dataclass
class TrainerCfg:
    num_epochs: int = 10_000
    batch_size: int = 2048
    learning_rate: float = 1e-3
    lr_decay_factor: float = 0.1 ** (1 / 1_000)  # Decay by 0.1 every 1000 epochs
    eval_freq: int = 10
    data_split: float = 0.95
    device: torch.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class Trainer:
    def __init__(self) -> None:
        self.trainer_cfg = TrainerCfg()
        self.model_cfg = TransformerCfg(type=args.model_type)

        self.dataset = GPTDataset(
            file_path=args.data_path,
            action_chunk_size=self.model_cfg.action_chunk_size,
            obs_history_len=self.model_cfg.history,
        )

        data_split_len = int(self.trainer_cfg.data_split * len(self.dataset))
        train_data, eval_data = torch.utils.data.random_split(
            self.dataset,
            [
                data_split_len,
                len(self.dataset) - data_split_len,
            ],
        )
        logger.info(
            f"Dataset split into {data_split_len} training samples and {len(self.dataset) - data_split_len} evaluation samples."
        )

        self.train_dataloader = DataLoader(
            train_data,
            batch_size=self.trainer_cfg.batch_size,
            shuffle=True,
        )
        self.eval_dataloader = DataLoader(
            eval_data,
            batch_size=self.trainer_cfg.batch_size,
            shuffle=False,
        )
        Model = ACT if self.model_cfg.type == "ACT" else GPT
        self.model = Model(config=self.model_cfg).to(self.trainer_cfg.device)

        self.optimizer = torch.optim.AdamW(
            self.model.parameters(),
            lr=self.trainer_cfg.learning_rate,
        )

        self.start_epoch = 0
        if args.resume:
            checkpoint = torch.load(args.resume, map_location=self.trainer_cfg.device)
            self.model.load_state_dict(checkpoint["model"])
            self.optimizer.load_state_dict(checkpoint["optimizer"])
            self.start_epoch = checkpoint["epoch"] + 1

        # Set up output path
        run_name = f"BC_{self.model_cfg.type}_{datetime.now().strftime('%m%d_%H%M%S')}"
        self.output_path = Path.cwd() / "outputs" / "BC" / "models" / run_name
        self.output_path.mkdir(parents=True, exist_ok=True)

        self.logger = WandbLogger(
            exp_name=run_name,
            base_path=self.output_path,
            config={**asdict(self.trainer_cfg), **asdict(self.model_cfg)},
        )

    def train(self) -> None:
        best_eval_loss = float("inf")
        for epoch in tqdm(
            range(self.start_epoch, self.trainer_cfg.num_epochs),
            desc="Training",
            dynamic_ncols=True,
            leave=True,
        ):
            self.model.train()
            self.update_learning_rate(iter=epoch)
            total_train_loss = 0.0
            num_batches = 0

            for batch in self.train_dataloader:
                observations = batch["observations"].to(self.trainer_cfg.device)
                target_actions = batch["actions"].to(self.trainer_cfg.device)
                target_eef_pos = batch["eef_pos"].to(self.trainer_cfg.device)

                output = self.model(observations)

                target_actions_unfold = target_actions.unfold(
                    1, self.model_cfg.action_chunk_size, 1
                ).permute(0, 1, 3, 2)

                loss = F.l1_loss(
                    input=output["actions"],
                    target=target_actions_unfold,
                )

                if self.model_cfg.predict_eef_pos:
                    target_eef_pos_unfold = target_eef_pos.unfold(
                        1, self.model_cfg.action_chunk_size, 1
                    ).permute(0, 1, 3, 2)

                    loss += F.l1_loss(
                        input=output["eef_pos"],
                        target=target_eef_pos_unfold,
                    )

                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                total_train_loss += loss.item()
                num_batches += 1

            avg_train_loss = total_train_loss / num_batches
            self.logger.log(
                {
                    "train/loss": avg_train_loss,
                    "train/learning_rate": self.optimizer.param_groups[0]["lr"],
                },
                step=epoch,
            )

            if epoch % self.trainer_cfg.eval_freq == 0:
                self.model.eval()
                eval_loss = 0.0
                with torch.no_grad():
                    for batch in self.eval_dataloader:
                        observations = batch["observations"].to(self.trainer_cfg.device)
                        target_actions = batch["actions"].to(self.trainer_cfg.device)
                        target_eef_pos = batch["eef_pos"].to(self.trainer_cfg.device)

                        output = self.model(observations)

                        target_actions_unfold = target_actions.unfold(
                            1, self.model_cfg.action_chunk_size, 1
                        ).permute(0, 1, 3, 2)

                        batch_loss = F.l1_loss(
                            input=output["actions"],
                            target=target_actions_unfold,
                        )

                        if self.model_cfg.predict_eef_pos:
                            target_eef_pos_unfold = target_eef_pos.unfold(
                                1, self.model_cfg.action_chunk_size, 1
                            ).permute(0, 1, 3, 2)

                            batch_loss += F.l1_loss(
                                input=output["eef_pos"],
                                target=target_eef_pos_unfold,
                            )

                        eval_loss += batch_loss.item()

                eval_loss /= len(self.eval_dataloader)

                # Save latest model
                torch.save(
                    {
                        "model": self.model.state_dict(),
                        "optimizer": self.optimizer.state_dict(),
                        "epoch": epoch,
                    },
                    self.output_path / "latest_model.pth",
                )

                # Save best model
                if eval_loss < best_eval_loss:
                    best_eval_loss = eval_loss
                    torch.save(
                        {
                            "model": self.model.state_dict(),
                            "optimizer": self.optimizer.state_dict(),
                            "epoch": epoch,
                        },
                        self.output_path / "best_model.pth",
                    )

                self.logger.log(
                    {
                        "eval/loss": eval_loss,
                        "eval/best_loss": best_eval_loss,
                    },
                    step=epoch,
                )

                tqdm.write(
                    f"Epoch {epoch}: eval loss={eval_loss:.4f}, best loss={best_eval_loss:.4f}"
                )

        # Save final model
        torch.save(
            {
                "model": self.model.state_dict(),
                "optimizer": self.optimizer.state_dict(),
                "epoch": epoch,
            },
            self.output_path / "final_model.pth",
        )
        self.logger.finish()

    def update_learning_rate(self, iter: int) -> None:
        """Update learning rate using exponential decay schedule."""
        if self.trainer_cfg.lr_decay_factor == 1.0:
            return
        lr = self.trainer_cfg.learning_rate * (self.trainer_cfg.lr_decay_factor**iter)
        for param_group in self.optimizer.param_groups:
            param_group["lr"] = lr


def main():
    trainer = Trainer()
    trainer.train()


if __name__ == "__main__":
    main()
