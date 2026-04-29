## base
You need model, can download from https://github.com/Qengineering/Qwen3-VL-4B-NPU
```bash
# clone
git clone https://github.com/airockchip/rknn-llm && cd rknn-llm

# install
sudo install rkllm-runtime/Linux/librkllm_api/aarch64/librkllmrt.so /usr/lib
sudo install rkllm-runtime/Linux/librkllm_api/include/rkllm.h /usr/include

# g++
g++ -std=c++11 ./examples/rkllm_api_demo/deploy/src/llm_demo.cpp -o llm_demo -lrkllmrt

# npu 监控
sudo watch -n 0.5 cat /sys/kernel/debug/rknpu/*

# 文本输入
./llm_demo /home/cat/llm/qwen3-vl-4b/qwen3-vl-4b-instruct_w8a8_rk3588.rkllm 2048 4096
```

多模态输入
```bash
# build
cd ~/llm/rknn-llm/examples/multimodal_model_demo/deploy/build 
cmake ..
make -j4

cd ~/llm/rknn-llm/examples/multimodal_model_demo/deploy/build
./demo /home/cat/llm/rknn-llm/datasets/000000000009.jpg /home/cat/llm/qwen3-vl-4b/qwen3-vl-4b-vision_rk3588.rknn  /home/cat/llm/qwen3-vl-4b/qwen3-vl-4b-instruct_w8a8_rk3588.rkllm 2048 4096 3

# or 2b
./demo /home/cat/llm/rknn-llm/datasets/9ebf5e905d046a565535e80d406c51f2.jpg /home/cat/llm/qwen3-vl-2b/qwen3-vl-2b-vision_rk3588.rknn  /home/cat/llm/qwen3-vl-2b/qwen3-vl-2b-instruct_w8a8_rk3588.rkllm 2048 4096 3
```

flask
```bash
# Usage: ./build_rkllm_server_flask.sh --workshop [RKLLM-Server Working Path] --model_path [Absolute Path of Converted RKLLM Model on Board] --platform [Target Platform: rk3588/rk3576] [--lora_model_path [Lora Model Path]] [--prompt_cache_path [Prompt Cache File Path]]
cd /home/cat/llm/rknn-llm/examples/rkllm_server_demo/
./build_rkllm_server_flask.sh --workshop /home/cat/llm/rknn-llm/examples/rkllm_server_demo/rkllm_server --model_path /home/cat/llm/qwen3-vl-4b/qwen3-vl-4b-instruct_w8a8_rk3588.rkllm --platform rk3588
```