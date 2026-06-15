#!/usr/bin/env python3
# encoding: utf-8
import os
import dashscope
from pathlib import Path

###Offline###
offline_llm = 'qwen3:1.7b'
if os.environ["ASR_LANGUAGE"] == 'Chinese':
    offline_asr = 'sherpa-onnx-streaming-zipformer-zh-xlarge-int8-2025-06-30'
    offline_tts = 'matcha-icefall-zh-baker'
else:
    offline_asr = 'sherpa-onnx-streaming-zipformer-en-2023-06-21'
    offline_tts = 'vits-ljs'
offline_tts_speaker = 100
offline_punct_model = 'sherpa-onnx-punct-ct-transformer-zh-en-vocab272727-2024-04-12'
ollama_host = 'http://localhost:11434'
sherpa_onnx_path = os.path.join(Path.home(), 'third_party/sherpa-onnx')
###

###Only in China###
# 阶跃星辰key
stepfun_api_key = ''
stepfun_base_url = 'https://api.stepfun.com/v1'
stepfun_llm_model = 'step-1v-8k'
#'step-1v-8k'/'step-1o-vision-32k'/'step-1.5v-mini'
stepfun_vllm_model = 'step-1o-vision-32k'

# 阿里云key
aliyun_api_key = ''
aliyun_base_url = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
aliyun_llm_model = 'qwen-max-latest'#'qwen-turbo'#'qwen-max-latest'
aliyun_vllm_model = 'qwen-vl-max-latest'
aliyun_tts_model = 'sambert-zhinan-v1'
aliyun_asr_model = 'paraformer-realtime-v2'
aliyun_voice_model = ''
######

###Internationally###
vllm_api_key = ''
vllm_base_url = 'https://openrouter.ai/api/v1'
vllm_model = 'qwen/qwen2.5-vl-72b-instruct'

llm_api_key = ''
llm_base_url = 'https://api.openai.com/v1'
llm_model = 'gpt-4o-mini'
openai_vllm_model = 'gpt-4o'
openai_tts_model = 'tts-1'
openai_asr_model = 'whisper-1'
openai_voice_model = 'onyx'
######

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    # The actual key used for invocation(实际调用的key)
    api_key = aliyun_api_key
    dashscope.api_key = aliyun_api_key
    base_url = aliyun_base_url
    asr_model = aliyun_asr_model
    tts_model = aliyun_tts_model
    voice_model = aliyun_voice_model
    llm_model = aliyun_llm_model
    vllm_model = aliyun_vllm_model
else:
    api_key = llm_api_key
    os.environ["OPENAI_API_KEY"] = api_key
    base_url = llm_base_url
    asr_model = openai_asr_model
    tts_model = openai_tts_model
    voice_model = openai_voice_model

# Get the path of the current program(获取程序所在路径)
code_path = os.path.abspath(os.path.split(os.path.realpath(__file__))[0])

if os.environ["ASR_LANGUAGE"] == 'Chinese':
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline')
    else:
        audio_path = os.path.join(code_path, 'resources/audio')
else:
    if os.environ["ASR_MODE"] == 'offline':  
        audio_path = os.path.join(code_path, 'resources/audio/offline/en')
    else:
        audio_path = os.path.join(code_path, 'resources/audio/en')

# Path to the recorded audio(录音音频的路径)
recording_audio_path = os.path.join(audio_path, 'recording.wav')

# Path to the synthesized (TTS) audio(语音合成音频的路径)
tts_audio_path = os.path.join(audio_path, "tts_audio.wav")

# Path to the startup audio(启动音频的路径)
start_audio_path = os.path.join(audio_path, "start_audio.wav")

# Path to the wake-up response audio(唤醒回答音频的路径)
wakeup_audio_path = os.path.join(audio_path, "wakeup.wav")

# Path to the error audio(出错音频的路径)
error_audio_path = os.path.join(audio_path, "error.wav")

# Path to the audio played when no sound is detected(没有检测到声音时音频的路径)
no_voice_audio_path = os.path.join(audio_path, "no_voice.wav")

# Path to the audio played when recording is complete(录音完成时音频的路径)
dong_audio_path = os.path.join(audio_path, "dong.wav")

record_finish_audio_path = os.path.join(audio_path, "record_finish.wav")

start_track_audio_path = os.path.join(audio_path, "start_track.wav")

track_fail_audio_path = os.path.join(audio_path, "track_fail.wav")
