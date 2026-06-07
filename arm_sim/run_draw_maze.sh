#!/usr/bin/env bash
#SBATCH --job-name=maze_draw
#SBATCH --partition=gpu
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --output=/data/maoting/Maze/arm_sim/logs/draw_maze_%j.log
#SBATCH --error=/data/maoting/Maze/arm_sim/logs/draw_maze_%j.log

set -e

# Isaac Sim EULA 자동 동의
export OMNI_KIT_ACCEPT_EULA=YES

# conda 환경 활성화
source /data/maoting/miniconda3/etc/profile.d/conda.sh
conda activate dexbench

echo "[env] python: $(which python)"
echo "[env] conda: $CONDA_PREFIX"
echo "[env] node: $(hostname)"
echo "[env] GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'N/A')"

cd /data/maoting/Maze
python arm_sim/draw_maze.py
