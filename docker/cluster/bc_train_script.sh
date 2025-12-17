#!/bin/bash

# #SBATCH --ntasks=1
#SBATCH --cpus-per-task=16
#SBATCH --gpus=rtx_4090:1
# #SBATCH --gpus=a100-pcie-40gb:1
# #SBATCH --gpus=a100_80gb:1
#SBATCH --mem-per-cpu=2000M
#SBATCH --time=47:00:00
#SBATCH --job-name="TASER BC Training"

module purge
module load eth_proxy

# Activate the venv
source /cluster/home/kjalloul/conda/bin/activate
conda activate taser

cd /cluster/home/kjalloul/taser

export PYTHONUNBUFFERED=1

python src/taser/cli.py training bc train \
    --model_type GPT \
    --data_path $SCRATCH/taser/bc_dataset.h5
