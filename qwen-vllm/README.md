# ── Qwen vLLM Server ──────────────────────────────────────────────────────────
#
# Runs Qwen2.5-Coder-3B-Instruct-AWQ behind an OpenAI-compatible REST API.
# Tuned for an NVIDIA RTX 3060 (6 GB VRAM).
#
# Prerequisites
# -------------
#   • Docker Engine + Docker Compose v2
#   • NVIDIA Container Toolkit  (nvidia-ctk runtime configure --runtime=docker)
#   • nvidia-smi working inside Docker (verify with: docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi)
#
# Quick Start
# -----------
#   cd qwen-vllm
#   docker compose up -d          # first run downloads ~2.5 GB model from HF
#   docker compose logs -f        # watch for "Application startup complete"
#
# Test the API
# ------------
#   curl -X POST "http://localhost:8000/v1/chat/completions" \
#        -H "Content-Type: application/json" \
#        -d '{
#              "model": "Qwen/Qwen2.5-Coder-3B-Instruct-AWQ",
#              "messages": [{"role": "user", "content": "List all active ROS 2 nodes with a bash command."}]
#            }'
#
# Point ROSA at this container
# ----------------------------
#   from rosa import ROSA
#   from langchain_openai import ChatOpenAI
#
#   local_llm = ChatOpenAI(
#       model="Qwen/Qwen2.5-Coder-3B-Instruct-AWQ",
#       openai_api_key="sk-no-key-required",   # vLLM ignores the key
#       openai_api_base="http://localhost:8000/v1",
#   )
#   agent = ROSA(ros_version=2, llm=local_llm)
#   agent.invoke("List all topics with publishers")
#
# VRAM Flags (RTX 3060 @ 6 GB)
# -----------------------------
#   --quantization awq            → 4-bit weights, ~2.5 GB VRAM footprint
#   --max-model-len 8192          → caps context window (prevents silent OOM)
#   --gpu-memory-utilization 0.85 → reserves 15 % for kwin_wayland / desktop
#
# Stop & remove container
# -----------------------
#   docker compose down
