# Polygon Separation · SAT 几何计算与批量查询实验

C++ 多边形分离向量实验，当前默认分支为 `SAT`。优化版本缓存边法线、自身投影、交叉投影与中心点；对固定多边形的多次平移查询复用预计算，并用 std::thread 分段处理查询。

## 实现入口

- [demos/Solution.cpp](demos/Solution.cpp)：预计算和多线程版本，当前线程数宏为 16。
- [demos/Solution_origin.cpp](demos/Solution_origin.cpp)：对照实现。
- [run_and_test.sh](run_and_test.sh)：编译并经 Runner/Interactor 执行交互协议。
- [data/](data/)：practice_1 到 practice_7 的输入与标准输出。
- [compare.sh](compare.sh)：仅比较 practice_1_std.out 与 sample.out，属于精确文本比较。

每个查询在线性扫描两组法线时使用平移投影修正，避免对所有顶点重复投影。当前代码每次创建一组线程，不能称为持久化线程池。

## 构建与运行

推荐在 Linux/POSIX 环境使用 Bash、支持 C++11 线程的 G++。仓库自带 Runner 与 Interactor；需确认二进制适配目标平台。

```sh
bash run_and_test.sh demos/Solution.cpp data/practice_1.in data/sample.out
bash compare.sh
```

程序输入包含 `OK` 握手，不能直接假定普通标准输入重定向等价于交互运行。脚本最终返回 0，验收时应另查 Runner 的实际结果与输出内容。

## 正确性与性能边界

- SAT 的边法线判定通常需要凸多边形前提；当前代码不检查凸性，也没有实现凹多边形分解。
- 完全包含、退化边、重复点和浮点边界需要专项验证，不能直接承诺所有输入上的最小分离向量正确。
- 已有两个计时文本记录 19.175 ms 和 15.428 ms，但没有完整环境、输入来源和重复试验元数据；本次不将它们转换为简历加速比。
- 本次核验了源码和脚本，没有在兼容的交互运行环境完成编译与全数据集验证。后续应记录编译器、线程数、输入散列、正确率与重复计时分布。

## 来源

仓库同时含多语言 demo。展示时应区分题目提供的交互框架、参考实现与本人优化内容；不将所有 demo 归为个人原创。
