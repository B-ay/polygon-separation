#!/bin/bash

# 文件比较脚本
# 用法: ./compare_files.sh

# 定义要比较的两个文件
FILE1="./data/practice_1_std.out"
FILE2="./data/sample.out"

# 检查文件1是否存在
if [ ! -f "$FILE1" ]; then
    echo "错误: 文件 $FILE1 不存在!"
    exit 1
fi

# 检查文件2是否存在
if [ ! -f "$FILE2" ]; then
    echo "错误: 文件 $FILE2 不存在!"
    exit 1
fi

# 使用 diff 命令比较文件内容
if diff -q "$FILE1" "$FILE2" > /dev/null 2>&1; then
    echo "✓ 文件内容一致"
    echo "$FILE1 和 $FILE2 的内容完全相同"
    exit 0
else
    echo "✗ 文件内容不一致"
    echo "$FILE1 和 $FILE2 的内容存在差异"
    
    # 可选: 显示详细的差异信息
    echo ""
    echo "差异详情:"
    echo "----------------------------------------"
    diff -u "$FILE1" "$FILE2"
    exit 1
fi