import os
from large_models.config import *
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode', default=1)
    enable_wakeup = LaunchConfiguration('enable_wakeup', default='true')
    awake_method = LaunchConfiguration('awake_method', default=os.environ['MIC_TYPE'])
    chinese_awake_words = LaunchConfiguration('chinese_awake_words', default='xiao3 huan4 xiao3 huan4')
    enable_setting = LaunchConfiguration('enable_setting', default='false')
    
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    enable_wakeup_arg = DeclareLaunchArgument('enable_wakeup', default_value=enable_wakeup) 
    awake_method_arg = DeclareLaunchArgument('awake_method', default_value=awake_method)
    awake_words_arg = DeclareLaunchArgument('chinese_awake_words', default_value=chinese_awake_words)
    enable_setting_arg = DeclareLaunchArgument('enable_setting', default_value=enable_setting)

    launch_list = [
            mode_arg,
            enable_wakeup_arg,
            awake_method_arg,
            awake_words_arg,
            enable_setting_arg,
            ] 

    
    if os.environ["ASR_MODE"] == 'offline':
        if os.environ["ASR_LANGUAGE"] == 'Chinese':
            asr_server = ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    (
                        f'''
                        exec python3 {sherpa_onnx_path}/python-api-examples/streaming_server.py \
                        --tokens={sherpa_onnx_path}/{offline_asr}/tokens.txt \
                        --encoder={sherpa_onnx_path}/{offline_asr}/encoder.int8.onnx \
                        --decoder={sherpa_onnx_path}/{offline_asr}/decoder.onnx \
                        --joiner={sherpa_onnx_path}/{offline_asr}/joiner.int8.onnx \
                        --doc-root={sherpa_onnx_path}/python-api-examples/web \
                        > /dev/null 2>&1
                        '''
                    )
                ],
                name='asr_server',
                shell=False,
                output='screen',
            )
        else:
            asr_server = ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    (
                        f'''
                        exec python3 {sherpa_onnx_path}/python-api-examples/streaming_server.py \
                        --tokens={sherpa_onnx_path}/{offline_asr}/tokens.txt \
                        --encoder={sherpa_onnx_path}/{offline_asr}/encoder-epoch-99-avg-1.onnx \
                        --decode={sherpa_onnx_path}/{offline_asr}/decoder-epoch-99-avg-1.onnx \
                        --joiner={sherpa_onnx_path}/{offline_asr}/joiner-epoch-99-avg-1.onnx \
                        --doc-root={sherpa_onnx_path}/python-api-examples/web \
                        > /dev/null 2>&1
                        '''
                    )
                ],
                name='asr_server',
                shell=False,
                output='screen',
            )
        launch_list.append(asr_server)

    vocal_detect_node = Node(
        package='large_models',
        executable='vocal_detect',
        output='screen',
        parameters=[{"port": "/dev/ring_mic",
                     # "port": "/dev/ttyCH341USB0",         
                     "mic_type": "mic6_circle",
                     "awake_method": awake_method,
                     "awake_word": chinese_awake_words,
                     "enable_setting": enable_setting,
                     "enable_wakeup": enable_wakeup,
                     "mode": mode,
                     "punct_model": f"{sherpa_onnx_path}/{offline_punct_model}/model.onnx"}]
    )
    launch_list.append(vocal_detect_node)
    return launch_list

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. (创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
