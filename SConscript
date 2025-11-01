import os
from rt_thread_tools import SConscript_source

# 获取当前脚本所在的目录
cwd = os.path.dirname(os.path.realpath(__file__))

# 定义源文件和头文件路径
src = SConscript_source(os.path.join(cwd, 'src'), '*.c')
inc = [os.path.join(cwd, 'inc')]

# 获取当前构建目标
group = GetCurrentRequest()

# 将源文件和头文件路径添加到构建目标中
group.extend(src)
group.extend(inc)

# 如果开启了示例代码，则添加示例代码的构建
if GetOption('NBM7100_SAMPLE'):
    # 定义示例代码的 SConscript 路径
    sample_sconscript = os.path.join(cwd, 'samples', 'SConscript')
    # 执行示例代码的 SConscript
    group.AddSubdirs(sample_sconscript)