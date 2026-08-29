# 机器人 AI 前沿文献综述与 EdgeGrasp 路线图（截至 2026-08-27）

> 检索截止：2026-08-27（Asia/Shanghai）
> 文档性质：一手来源优先的中文技术综述与项目路线建议
> 项目范围：SO-101 仿真、ROS 2 Jazzy、MoveIt plan-only、typed trajectory gate、目标/时钟新鲜度、安全停止、APPROACH → DESCEND → CLOSE_GRIPPER → LIFT
> 重要边界：本文没有运行 ROS、Gazebo 或真实硬件，没有复现外部模型，也没有改变任何现有安全合同。论文或厂商报告中的结果均不是本项目的实际采用情况或物理抓取证据。

## 0. 调研计划与分类框架

本次调研按“从 EdgeGrasp 当前闭环向外扩展”的方式组织，而不是按模型热度罗列。

1. **先固定项目事实与主张边界**：只读核对 [项目 README](../README.md)、[验证报告](validation-report.md)、[仿真计划](simulation-plan.md) 及运行观察；区分软件协议、仿真物理观察、真实硬件证据。
2. **近场路线**：视觉抓取、6D pose、affordance、闭环视觉伺服、动态目标与延迟、端侧部署、运动规划/控制接口、sim-to-real、抓取评测、运行时安全。
3. **宏观路线**：VLA/机器人基础模型、Diffusion/Flow Policy、模仿学习与离线 RL、World Model、通用/移动操作、多机器人数据、合成数据、触觉、多模态、具身推理和安全。
4. **状态审计**：对 2025–2026 工作分别核对 arXiv 首发、正式 proceedings/期刊状态、公司报告或产品页，以及代码、权重、数据、配方和许可证是否真的公开。
5. **综合吸收**：建立技术谱系，比较路线分歧、证据强弱和瓶颈；将可行增量映射到 EdgeGrasp 的 typed、安全、可回放结构。
6. **个人路线**：结合半年准备期和嵌入式/中间件背景，比较算法、端侧 AI、机器人系统与基础模型研究岗位的现实匹配度。

### 证据标签

| 标签 | 含义 | 本文如何使用 |
|---|---|---|
| **PR** | 有可核验正式 proceedings 或期刊版本的同行评审论文 | 可作为方法与已披露实验的较强证据；仍不等于本项目复现 |
| **PP** | arXiv/preprint，未核验正式同行评审版本 | 作为趋势和待验证假设，不作为成熟工程结论 |
| **TR** | 公司或研究机构技术报告 | 报告作者/公司自有系统证据，需警惕闭源、选择性基准与复现不足 |
| **Product** | 官方产品页、模型卡或发布公告 | 仅说明产品方公开主张和限制，不等同论文证据 |
| **Docs/System** | 官方文档、标准、代码仓库或数据项目 | 用于接口语义、开放状态和工程能力核对 |

“开放”在本文中拆成代码、权重、数据、训练配方和许可证五件事；只公开其中一项，不写成“完全开源”。所有论文数字均视为**论文/项目方在其协议下的报告**，除非另有独立复现来源。

---

## 1. 一页式结论

### 1.1 最重要的十点

1. **EdgeGrasp 当前最有价值的资产是可审计执行边界，不是抓取模型。** 现有路径将感知目标、tf2/IK、MoveIt plan-only、trajectory validation、typed gate 和 FJT 分开，并对错误时间域、过期/未来目标、阶段错配和晚到结果 fail closed。这正是学习模型接入机器人时最容易缺失的工程层。
2. **当前没有物理抓取成功证据。** 最新候选虽出现双侧接触和 3.711 mm 瞬时抬升，但接触提前 1.737 s 结束、无保持，结论仍是 <code>physics_grasp_verified=false</code>。308 个 Windows 测试、49 个 ROS safety 测试，以及分别为 60 个无依赖与 42 个 Jazzy action/client 的四阶段测试，证明各自软件合同，不证明物体被稳定抓取。
3. **近半年最值得做的 AI 增量不是 VLA，而是“可测量的 RGB-D/6D 感知 → typed candidate adapter”。** Contact-GraspNet、FoundationPose、GraspGen 一类模型可以产生候选 pose/width/score；它们不应直接发布控制器命令。
4. **动态抓取的核心不是模型 FPS，而是时间一致性。** 应显式记录 capture、inference、tf2、prediction-at-execution、plan、gate、send 和 controller terminal 的时间链，比较无补偿、常速度预测和短时闭环三种方案。
5. **视觉伺服和 Diffusion/Flow Policy 都适合短时滚动，但会引入新的控制权问题。** 每个动作块都必须完整、短时有效、带时间戳且通过现有 gate；推理超时、半成品 stream、目标过期或阶段不一致均应停止。
6. **VLA 的强项是语义、任务分解、跨任务先验；弱项仍是精确 3D、接触、延迟尾部、跨 embodiment 标定和可验证安全。** 最可靠的工程形态是“VLA/策略建议 → typed adapter → 显式几何/规划 → 独立安全执行”。
7. **当前最强的公开证据仍以窄任务、特定机器人和作者协议为主。** OXE、DROID、RoboMIND 等扩大了数据覆盖，Octo、OpenVLA、RDT、π0/π0.5 等推进通用策略，但没有哪项工作证明了任意机器人、任意家庭环境或 fail-safe 通用操作。
8. **Sim-to-real 的关键正在从“随机化更多”转向“配对校准、现实数据约束和失败覆盖”。** 仿真应提供可重复扰动与反例生成；只有同一协议下的 sim/real 配对结果才能说明仿真排序是否有现实意义。
9. **未来 2–3 年最可能形成的主流是双时间尺度系统。** 慢速语义/世界模型负责目标、子任务和候选，快速局部策略或伺服负责接触响应，外部 runtime assurance 负责限位、新鲜度、碰撞和停止。这是基于近年论文与公开系统的推断，不是已确定事实。
10. **对有嵌入式/电视中间件经验、半年准备期的人，最佳切入点是端侧感知部署 + 机器人系统可靠性。** 先把 C++/Python/Linux、ROS 2/DDS、时间戳、QoS、日志回放、ONNX/TensorRT、watchdog 和 fail-closed 做成量化作品；纯 VLA/World Model 研究不宜作为唯一主线。

### 1.2 对 EdgeGrasp 的推荐架构

    RGB-D / tag / 6D pose / grasp model
                     │
                     ▼
    typed candidate: frame + capture stamp + epoch
      + pose/width/score/uncertainty/model digest
                     │
                     ▼
    freshness / frame / phase / bounds / NaN checks
                     │
                     ▼
    tf2 + IK + MoveIt plan-only + trajectory validation
                     │
                     ▼
              existing typed gate
                     │
                     ▼
      FJT/controller ── independent contact/lift/retention observer

学习模型应处于上图的“候选/建议”侧。只有独立观察同时满足本项目定义的接触、至少 20 mm 抬升和完整保持窗口，才可改变物理抓取结论。

### 1.3 六个月的主线选择

| 路线 | 半年内产出可信度 | 与既有背景匹配 | 建议 |
|---|---:|---:|---|
| 端侧感知部署 + ROS 2 安全集成 | 高 | 高 | **主线**：静态 RGB-D/pose、ONNX/TensorRT、时间链、长稳、故障注入 |
| 3D 视觉/抓取候选算法 | 中 | 中 | **副主线**：复现一个轻量基线，重点做坐标、误差、延迟和失败分析 |
| 操作策略/IL（ACT、DP3、SmolVLA） | 中低 | 中 | 仿真离线对照即可；不要吞掉安全边界和评测时间 |
| VLA/World Model 基础研究 | 低 | 低 | 阅读并做接口理解，不作为半年唯一求职赌注 |

---

## 2. EdgeGrasp 当前事实、已证据与未证据

### 2.1 当前已观察事实

根据 [项目 README](../README.md) 与 [验证报告](validation-report.md) 的当前快照：

- 平台为 SO-101 仿真与 ROS 2 Jazzy；MoveIt 只负责规划，执行需经过 trajectory validation 和 typed gate。
- 四阶段链为 APPROACH → DESCEND → CLOSE_GRIPPER → LIFT；每阶段绑定同一任务语义下最新且新鲜的 <code>TrackedTarget</code>，而不是长期复用任务初始 pose。
- 目标或规划超过 200 ms 新鲜度边界、时钟回退、错误 frame/domain/epoch、future/out-of-order 目标、结果相关性错误等条件会阻止下游派发。
- 当前稳定快照记录 308/308 Windows CPython 项目测试、49/49 <code>edgegrasp_ros</code> Jazzy 测试、60/60 无依赖四阶段测试与 42/42 Jazzy sequence 测试；这些是不同子套件的计数，不应简单相加为一个覆盖率数字。
- [Candidate011 运行观察](observations/2026-08-27-candidate011-q0p40-runtime.md) 是最强的瞬时物理信号：0.463 s 双侧接触、3.711 mm 峰值抬升；接触在完成前 1.737 s 消失，物体未保持。

### 2.2 证据边界

| 可以声称 | 不能据此声称 |
|---|---|
| 指定版本、指定测试协议下的软件合同通过 | 真实机械臂已安全、稳定地抓住物体 |
| MoveIt 生成了满足当前 validator 的 plan | 全身执行必然无碰撞 |
| typed gate 接受并关联了命令/结果 | 感知目标正确、物体被抓住 |
| FJT 返回 terminal success | 夹爪形成力闭合或有保持 |
| watchdog/cancel 请求按软件协议触发 | 电机或机械体已经物理停止 |
| Gazebo 中出现接触或瞬时抬升 | sim-to-real 已成立 |

因此本文所有“可转化增量”都必须保持：**模型输出不可直接越过现有安全合同；软件测试通过不可写成物理抓取成功。**

---

## 3. 技术谱系：从几何抓取到具身基础模型

机器人操作前沿不是一条替代链，而是五层能力逐渐叠加：

1. **几何与解析抓取**：Dex-Net、GraspNet、Contact-GraspNet 用点云、碰撞和解析质量指标产生候选。
2. **对象/区域语义**：6D pose、affordance 和 VLM keypoint 方法回答“抓哪里、以什么姿态、对哪个部件操作”。
3. **闭环控制**：视觉伺服、receding-horizon、Diffusion/Flow Policy 回答“观测变化后如何连续修正”。
4. **数据与通用策略**：OXE、DROID 等扩大机器人数据；RT-1/2、Octo、OpenVLA、RDT、π0 将语言、视觉和动作统一建模。
5. **世界模型与安全外壳**：视频/latent world model尝试预测后果；Simplex/SOTER、CBF、runtime monitoring 和显式 gate 负责把不可信策略限制在可接受集合内。

路线之间的合理关系是：

    language / VLM reasoning
              ↓ proposes goal, object, affordance
    pose / grasp / policy model
              ↓ proposes pose or short action chunk
    planner / servo / controller
              ↓ produces executable command
    runtime assurance
              ↓ accepts, rejects, stops, records evidence
    independent outcome observer

“端到端”模型减少了手工模块，但没有消除标定、时间、动力学、机械限位、接触和停止语义；这些问题只是从显式接口转移到数据分布和系统外壳中。

---

## 4. 与 EdgeGrasp 直接相关的路线

### 4.1 视觉抓取、6D pose 与 affordance

| 代表工作 | 年份/状态 | 问题与方法直觉 | 主要证据与开放情况 | 局限 | 可转化的 EdgeGrasp 增量 |
|---|---|---|---|---|---|
| [Dex-Net 2.0](https://roboticsproceedings.org/rss13/p58.html) | 2017，RSS，PR | 用大规模合成点云/抓取与 GQ-CNN 估计平行夹爪鲁棒性 | 论文报告 6.7M 合成样本和 1,000+ 真实试验；代码/数据体系公开 | 解析摩擦/几何质量、传感器与桌面假设限制明显 | 建立 analytic score 与实际 contact/lift/retention 的分层对照 |
| [GraspNet-1Billion](https://openaccess.thecvf.com/content_CVPR_2020/papers/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.pdf) / [项目](https://graspnet.net/) | 2020，CVPR，PR | 用统一 RGB-D 数据与 6D 平行夹爪候选评测杂乱场景抓取 | 论文版与后续官网版数据量不同；数据/API/基线公开 | analytic collision/quality AP 不是实际抬升保持；版本数字不可混写 | 离线候选生成器；输出 pose/width/score/source stamp |
| [Contact-GraspNet](https://arxiv.org/abs/2103.14127) | 2021，ICRA，PR | 从局部点云直接预测以可见接触点为根的 6-DoF 抓取分布 | 约 17M 仿真抓取训练；官方代码公开；论文有真实杂乱场景试验 | 固定相机、平行夹爪、点云完整性与遮挡偏差 | 作为首个学习型候选基线；严禁直接发 FJT |
| [AnyGrasp](https://arxiv.org/abs/2212.08333) / [SDK](https://github.com/NYU-robot-learning/anygrasp) | 2023，IEEE T-RO，PR | 密集 7-DoF 抓取并做时序抓取关联、稳定性与运动预测 | 作者报告动态/清箱结果；SDK 需注册许可，并非完全开放 | gripper 遮挡、反馈不足、平台和协议依赖；作者成功率不可外推 | 用运动预测形成 <code>target_at_execution_time</code>，保留 epoch/置信度/stale reject |
| [MegaPose](https://arxiv.org/abs/2212.06870) / [代码](https://github.com/megapose6d/megapose6d) | 2022，CoRL，PR | 给定 CAD/ROI，以大量合成训练和 render-and-compare 估计新物体 6D pose | 代码、模型与数据公开 | 依赖 CAD/ROI；对称、重遮挡和计算成本 | 已知物体 pose 基线，评估 pose error 与全链路延迟 |
| [FoundationPose](https://openaccess.thecvf.com/content/CVPR2024/html/Wen_FoundationPose_Unified_6D_Pose_Estimation_and_Tracking_of_Novel_Objects_CVPR_2024_paper.html) / [代码](https://github.com/NVlabs/FoundationPose) | 2024，CVPR Highlight，PR | 统一 CAD-based 与 reference-image 的新物体 pose/tracking | 官方代码与模型公开，但模型有 NVIDIA EULA | 需 mask/初始化；遮挡、对称、跟踪丢失与 GPU 延迟 | 在 <code>TrackedTarget</code> 增加 covariance、track age、model digest；不确定或过期即拒绝 |
| [Where2Act](https://openaccess.thecvf.com/content/ICCV2021/html/Mo_Where2Act_From_Pixels_to_Actions_for_Articulated_3D_Objects_ICCV_2021_paper.html) | 2021，ICCV，PR | 从点云预测可交互位置、动作类型和方向 | 项目、代码和数据公开；主要在 PartNet-Mobility/SAPIEN | 针对推/拉铰接物体，不是两指抓取；真实迁移有限 | 将单点目标扩展为 affordance region，但候选仍走 plan-only/gate |
| [MOKA](https://www.roboticsproceedings.org/rss20/p062.html) | 2024，RSS，PR | VLM 通过图像标记选择 affordance keypoint，再由模块化策略执行 | 真实机器人开放世界任务；项目材料公开 | VLM keypoint 会错，精确深度/姿态和安全依赖下游模块 | 只将 VLM 输出作为目标/区域建议，不允许自由文本成为命令 |
| [HRP: Human Affordances for Robotic Pre-Training](https://www.roboticsproceedings.org/rss20/p068.html) | 2024，RSS，PR | 从人类视频预训练 affordance 表征，再迁移到机器人 | 论文报告 3,000+ robot trials，并开放代码、权重和数据 | 人手与夹爪 embodiment gap、视频偏差和任务范围 | 作为 affordance ranking 离线对照，不改变执行合同 |
| [Pos3R](https://openaccess.thecvf.com/content/CVPR2025/papers/Deng_Pos3R_6D_Pose_Estimation_for_Unseen_Objects_Made_Easy_CVPR_2025_paper.pdf) | 2025，CVPR，PR | 借助 3D foundation feature 做训练自由 RGB-only 新物体 pose，再 refinement | 官方项目提供代码入口；在多个 pose 数据集评测 | RGB-only 与当前 RGB-D 路线不完全匹配；重遮挡仍难 | 后续对照项，不是首个半年集成目标 |
| [GraspGen](https://arxiv.org/abs/2507.13097) / [代码](https://github.com/NVlabs/GraspGen) | 2026，ICRA，PR；2025 首发 | Diffusion Transformer 生成 6D 抓取，判别器筛选；大规模合成抓取 | 论文摘要为 53M+，当前仓库/数据页为 57M+；代码与数据工具公开 | 数据口径随版本变化；生成分数不等于执行/保持/安全 | 可研究 SO-101 gripper adapter 与候选延迟，不得改写物理结论 |
| [GraspGen-X](https://openaccess.thecvf.com/content/CVPR2026/papers/Han_GraspGen-X_Cross-Embodiment_6-DOF_Diffusion-based_Grasping_CVPR_2026_paper.pdf) / [代码](https://github.com/NVlabs/GraspGenX) | 首发 2026-05-31；CVPR 2026，PR | 用夹爪 swept volume 条件化，学习跨夹爪 6D 抓取生成 | 项目页为 395M/350M sampled 口径，仓库为 >2B computed grasps/32 grippers；代码/checkpoint 公开，完整训练数据待发布且许可证不同 | 新夹爪 zero-shot 仍需几何、运动学和现实验证 | 研究“gripper geometry 作为 typed condition”，不是直接部署承诺 |

**判断。** 对 EdgeGrasp，最小且信息增益最高的比较是：经典 tag/已知 CAD pose → Contact-GraspNet 或 GraspGen 候选 → 同一 IK/plan-only/gate → 独立物理观察。FoundationPose 更适合已知/参考物体跟踪，但 GPU 延迟和 tracking-loss 必须成为 gate 输入。

### 4.2 闭环抓取、视觉伺服、动态目标与延迟

| 代表工作 | 年份/状态 | 方法与证据 | 局限 | EdgeGrasp 增量 |
|---|---|---|---|---|
| [Closing the Loop for Robotic Grasping](https://www.roboticsproceedings.org/rss14/p21.html) | 2018，RSS，PR | GG-CNN 以高频像素级质量、角度和宽度图做闭环 PBVS；真实动态/扰动试验 | 任务、相机和夹爪较窄；像素质量不是安全保证 | 建立短时重新观测/重选抓取的历史基线 |
| [Learning Hand-Eye Coordination](https://arxiv.org/abs/1603.02199) | 2018，IJRR 系统论文，PR | 大规模真实尝试学习抓取成功与连续视觉伺服 | 数据与硬件成本极高，平台特定 | 说明闭环价值，但不适合作为半年首个复现 |
| [Final-Phase IBVS](https://arxiv.org/abs/2001.05650) | 2020，公开论文 | 近距离深度失效时，从 RGB-D 特征切换至 RGB-only IBVS 完成 final approach | 依赖 eye-in-hand、特征可见和手眼标定；只覆盖最终阶段 | 在 DESCEND 后增加小范围、限速、短时 servo sandbox |
| [Dynamic Grasp and Trajectory Planning](https://link.springer.com/article/10.1007/s10514-018-9799-1) | 2018，Autonomous Robots，PR | 在线评估抓取候选；局部跟踪移动目标，不可达时切回全局规划 | 跟踪/速度模型和平台假设强 | 比较 latest sample、预测执行时 pose、重规划切换 |
| [Diffusion Policy](https://roboticsproceedings.org/rss19/p026.html) / [代码](https://github.com/real-stanford/diffusion_policy) | 2023，RSS，PR | 条件去噪生成多峰动作序列，以 receding horizon 执行短段 | 迭代采样延迟、demo 分布和 chunk 过期；没有内生安全保证 | 动作块先离线/仿真，记录 P50/P95、age、accept/reject |
| [Real-Time Execution of Action Chunking Flow Policies](https://arxiv.org/abs/2506.07339) | 2025，NeurIPS，PR | 异步生成下一动作块，冻结必然执行的前缀，并对其余部分 inpaint，缓和推理延迟和切换不连续 | 依赖策略校准；延迟实验不等于任意网络/硬件实时性 | 为异步策略定义 chunk version、observation stamp 和替换边界 |
| [Reactive Diffusion Policy](https://www.roboticsproceedings.org/rss21/p052.html) | 2025，RSS，PR | 慢速视觉 diffusion 提供计划，快速触觉反馈环修正接触 | 仅若干接触密集任务；需触觉硬件和低延迟同步 | 长期可在 CLOSE/LIFT 加滑移 observer；不得先跳过视觉/物理基线 |
| [Grasp-MPC](https://arxiv.org/abs/2509.06201) / [NVIDIA 论文页](https://research.nvidia.com/labs/lpr/publication/yamada2026graspmpc/) | 首发 2025-09-07；ICRA 2026，PR | 以合成成功/失败轨迹学视觉 value，再用 MPC/MPPI 滚动优化抓取轨迹 | 代码截至检索仍标 Coming soon；value 受合成标签限制 | 适合作为未来 advisory cost，不应掌握 gate 接受权 |
| [Jetson-PI](https://arxiv.org/abs/2607.12659) / [代码](https://github.com/PKU-SEC-Lab/Jetson-PI) | 首发 2026-07-14；PP | 针对 Jetson 上 VLA 低控制频率，使用异步推理和 foresight 对齐 | 未核验正式 venue；作者硬件/任务结果不具普适性 | 高度贴合 target age 与异步错配问题；先做延迟注入再考虑模型 |

建议将动态链路记录为：

    capture_stamp
      → inference_start / inference_end
      → tf2_lookup_stamp
      → prediction_target_stamp
      → plan_stamp
      → gate_check_stamp
      → FJT_send_stamp
      → controller_terminal_stamp
      → contact/lift/retention observation

最低指标集：目标年龄、时钟域/epoch、预测 horizon、丢帧数、端到端 P50/P95/P99、stale/future reject、阶段错配、safe-stop 请求与控制器/硬件确认时间。模型 FPS 不能代替这条链。

### 4.3 端侧感知、规划与控制接口

| 一手来源 | 核心语义 | 对 EdgeGrasp 的要求 |
|---|---|---|
| [TensorRT 官方文档](https://docs.nvidia.com/tensorrt/) | ONNX/PyTorch 等模型优化、量化与 NVIDIA GPU/Jetson 推理 | 比较原模型、ONNX、FP16/INT8 时同时报告精度、P50/P95、显存、温度和长稳；桌面 GPU 不能替代目标端 |
| [Isaac ROS NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) | ROS 2 type adaptation/negotiation 与同进程 zero-copy 加速 | zero-copy 只优化数据搬运；frame、source stamp、epoch 和目标身份仍需显式 typed 字段 |
| [MoveIt PlanningOptions](https://docs.ros.org/en/ros2_packages/rolling/api/moveit_msgs/msg/PlanningOptions.html) | <code>plan_only=true</code> 返回规划而不执行 | 保持当前“MoveIt 规划、EdgeGrasp 决定是否执行”的权责分离 |
| [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) | 末端/关节速度或 pose servo；提供奇异点、碰撞和限位能力 | 如增加 final servo，必须另定义控制权、限速、短超时、watchdog、停止 ownership；不得绕过 gate |
| [MoveIt Trajectory Execution](https://moveit.picknik.ai/main/api/html/trajectory_execution.html) | 管理轨迹执行、控制器选择与分段 | 说明规划结果和执行终端是不同证据层 |
| [ros2_control JointTrajectoryController Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) | FJT 可监控执行、tolerance、取消与时钟变化 | FJT success 只说明控制器合同；cancel requested 仍不等于物理停止 |

推荐接口：

    model output
      → typed candidate
      → freshness / epoch / frame / confidence checks
      → IK / plan-only / trajectory validation
      → typed gate
      → FJT

新增模型应记录：模型/权重 digest、输入 frame、capture/source/receive stamp、运行时版本、预处理配置、硬件、资源占用、温度、推理分位数和输出拒绝原因。

### 4.4 Sim-to-real 与抓取评测

| 工作/基准 | 年份/状态 | 证据价值 | 关键限制 | EdgeGrasp 用法 |
|---|---|---|---|---|
| [Domain Randomization](https://arxiv.org/abs/1703.06907) | 2017，IROS，PR | 合成视觉随机化可帮助检测器迁移，含真实抓取演示 | 外观随机化不覆盖相机外参、延迟、控制和接触 | 用于光照/纹理扰动，不写成完整 sim-to-real |
| [BayesSim](https://roboticsproceedings.org/rss15/p29.html) | 2019，RSS，PR | 从真实轨迹推断仿真参数后验，而非盲目均匀随机化 | 后验依赖观测充分性和 simulator model class | 用日志估计延迟、摩擦、控制周期分布 |
| [DROPO](https://gabrieletiboni.github.io/dropo/) | 2023，Robotics and Autonomous Systems，PR | 从少量离线真实轨迹估计动力学参数分布，再做 domain randomization | 依赖真实轨迹、动力学模型和可辨识性；不是视觉—执行全链证明 | 有硬件日志后估计摩擦/控制/延迟分布，当前只作设计参考 |
| [Reconciling Reality through Simulation](https://www.roboticsproceedings.org/rss20/p015.pdf) | 2024，RSS，PR | 用现实观测校准仿真，再训练更鲁棒的操作控制器 | 仍受 simulator model class、校准样本和任务范围限制 | 先现实测量再随机化，不盲目扩大扰动 |
| [Sim-and-Real Co-Training](https://www.roboticsproceedings.org/rss21/p109.pdf) | 2025，RSS，PR | 混合真实与仿真示范共训；作者协议报告多平台平均增益 | 相对增益依数据比例、平台、任务与基线，不能泛化 | 将来保留真实 holdout 和配对消融 |
| [Generalizable Domain Adaptation for Sim-and-Real Policy Co-Training](https://proceedings.neurips.cc/paper_files/paper/2025/hash/1185c89347a3f21ffc48c9d083c9437c-Abstract-Conference.html) | 2025，NeurIPS，PR | 以 OT/UOT 对齐 sim/real observation-action 分布 | 对齐可能掩盖物理差异；作者提升只对特定协议成立 | 作为表示对齐研究项，不能替代 freshness/gate |
| [BOP](https://bop.felk.cvut.cz/tasks/) / [2025 Challenge](https://bop.felk.cvut.cz/challenges/bop-challenge-2025/) | 持续基准，Docs/System | VSD/MSSD/MSPD 等处理遮挡与对称的 6D pose 评测；2025 增加工业数据集与多视图设置，model-free 路线自 2024 已纳入 | 只评 pose，不评夹爪接触、抬升和保持 | 给 pose 模块独立指标，避免用抓取结果反推 pose |
| [GraspNet evaluation](https://graspnet.net/evaluation.html) | benchmark，Docs/System | 统一 analytic friction/collision precision@k/AP | analytic success 不是现实 force closure | 候选排序离线指标，与仿真/真实 outcome 分开 |
| [REACH](https://sites.google.com/berkeley.edu/reach/home/dataset) | 真实抓取数据项目 | 2,625 次真实抓取和重复协议，适合 precision/recall/repeatability 设计 | 平台/物体受限 | 借鉴每物体重复、视频/JSON、失败标签 |
| [Get a Grip](https://sites.google.com/view/get-a-grip-dataset/home) | 2024，CoRL，PR | 3.5M 仿真抓取、4.3K 物体，含 hard negatives 和腕部扰动 | 模拟 pick success 不等于现实保持 | 为 near-failure 数据和 collision/perturbation 设计提供模板 |
| [FMB](https://functional-manipulation-benchmark.github.io/) | 2024，IJRR，PR | 真实多阶段操作、22,550 trajectories、程序化物体与 RGB-D | Panda/任务范围有限 | 按四阶段分别报告成功、失败与恢复，而非一个总成功率 |
| [SIMPLER](https://proceedings.mlr.press/v270/li25c.html) | CoRL 2024（PMLR 270，2025），PR | 用 1,500+ paired sim-real evaluations 检验仿真是否保持策略排序 | 相关性仅对特定平台、校准和任务成立 | 将来有硬件时做同配置 paired protocol；当前只能设计不能宣称 |
| [LIBERO](https://libero-project.github.io/research) | NeurIPS 2023，PR benchmark | 以多个 task suites 评估终身/多任务机器人学习；代码和任务公开 | 主要仿真，任务 success 对 reset、seed 和实现敏感 | 用作策略离线对照，不替代物理抓取评测 |
| [RoboDojo](https://robodojo-benchmark.com/) | 2026，PP/System | 官方系统报告仿真、真实任务与远程真实评测接口 | 截止日未核验 peer-reviewed venue；远程平台仍有协议/可用性限制 | 关注可复现实机评测趋势，当前不作为项目证据 |
| [RoboCasa365](https://proceedings.iclr.cc/paper_files/paper/2026/hash/a05003fdb1e9562ab0c0a9719ea4de10-Abstract-Conference.html) | 2026，ICLR，PR | 大量厨房原子/组合任务和 synthetic/human demos | 大规模仿真仍有视觉/接触域差距 | 借鉴阶段标签和扰动矩阵，不追求其规模 |

EdgeGrasp 推荐的评测合同：

- 分母必须固定并公开：对象、位姿、速度、光照、摩擦/材质、延迟、随机种子、模型/配置 digest。
- 将 pose、plan、gate、controller、contact、lift、retention 分成独立字段；任何上游 PASS 不自动传播为下游成功。
- 每个 cell 至少报告尝试数、成功数、Wilson 区间、失败分类和中止原因；样本很小时明确“不足以比较”。
- 端侧报告 P50/P95/P99、掉帧、温度、显存、长稳和 stop latency。
- 仿真成功写“指定 simulator/config 下的 outcome”；只有真实配对试验才能写真实成功。

### 4.5 安全执行与可验证 fail-closed 系统

| 工作 | 年份/状态 | 理念与证据 | 局限 | EdgeGrasp 关系 |
|---|---|---|---|---|
| [Simplex Architecture](https://experts.illinois.edu/en/publications/the-simplex-architecture-for-safe-on-line-control-system-upgrades/) | 1998，ACC，PR | 高性能控制器可更新；独立可靠基线控制器与决策模块保证切换 | 保证依赖系统模型、切换条件与 fallback 能力 | 学习模型是不可信高性能层；gate/stop 是独立边界 |
| [SOTER](https://arxiv.org/abs/1808.07921) | 2018，公开论文 | 将 runtime assurance 思想用于机器人：先进控制器 + 安全 fallback + monitor | 仿真/规格覆盖不等于整个真实系统认证 | 支持“模型建议、独立 gate 决定”的总体架构 |
| [SOTER on ROS](https://arxiv.org/abs/2008.09707) | 2020，Runtime Verification，PR | 用语言和 ROS 组件实现运行时安全切换 | middleware、OS、时钟和硬件停止仍是信任边界 | 可借鉴 monitor/contract 分离和故障注入 |
| [Measurement-Robust CBF](https://proceedings.mlr.press/v155/dean21a.html) | 2021，CoRL，PR | 将测量不确定性和采样误差显式纳入 barrier constraint | 需要可信动力学、状态估计和可计算安全集 | 先把 pose covariance 映射为 reject/保守走廊；不要声称已具 CBF 证明 |
| [VerifAI](https://doi.org/10.1007/978-3-030-25540-4_25) | 2019，CAV，PR | 以仿真引导搜索系统反例并做 falsification | 找不到反例不等于证明；simulator fidelity 限制 | 系统化生成 stale、rollback、late result、接触丢失场景 |
| [Runtime Verification and Field Testing for ROS](https://research.chalmers.se/en/publication/542744) | 2024，IEEE TSE 50(10):2544–2567，PR | 从文献、仓库和问卷总结 ROS runtime verification/fail-safe 实践 | 指南不是对某个项目的认证 | 支持日志、replay、fault injection、evidence ledger |
| [Safe-ROS](https://arxiv.org/abs/2511.14433) | 2025，FMAS / EPTCS 436，peer-reviewed proceedings | 智能子系统外置独立 Safety Instrumented Functions，并讨论形式验证边界 | 论文承认实时性、OS/Docker、ROS middleware 和组合性未全部建模 | 与 typed gate 相近；应学习其边界意识而非宣称形式安全 |
| [ISO 10218-1:2025](https://www.iso.org/standard/73933.html) | 2025，国际标准 | 工业机器人本体安全要求 | 适用范围、系统集成和合规评估严格；标准存在不等于项目符合 | 仅作真实硬件阶段的要求索引；当前项目无合规声明 |

必须保持以下语义：

- cancel requested ≠ robot physically stopped；
- FJT terminal success ≠ object grasped；
- gate pass ≠ whole-body collision-free；
- watchdog timeout ≠ mechanical braking completed；
- simulation contact ≠ force closure；
- 单个 monitor/形式组件 ≠ 全系统形式验证；
- 真实机械安全还需要控制器/硬件 acknowledgement、限速、工作空间、碰撞监测、实际 E-stop 和现场风险评估。

---

## 5. 更宏观的“机器人 + AI”前沿

### 5.1 VLA 与机器人基础模型

这一主线从“语言选择已有技能”演进到“视觉、语言、机器人状态和连续动作联合建模”。数据规模扩大带来语义与任务泛化，但低层几何、动作时效和安全外壳没有因此消失。

| 代表工作 | 年份/状态 | 问题与方法直觉 | 主要证据与开放情况 | 局限 | 与 EdgeGrasp 的关系 |
|---|---|---|---|---|---|
| [SayCan](https://proceedings.mlr.press/v205/ichter23a.html) | CoRL 2022，PR | 将 LLM 的语言先验与已有机器人 skill 的可执行性相乘，选择下一技能 | 真实移动操作；项目页公开，底层 skill 预定义 | 不是低层控制；可供性评分与 skill 覆盖决定上限 | LLM 只能提议 phase/skill，gate 决定是否执行 |
| [RT-1](https://roboticsproceedings.org/rss19/p025.html) / [代码](https://github.com/google-research/robotics_transformer) | RSS 2023，PR | 图像+语言 → 离散动作 token 的多任务 Transformer | 论文报告约 13 台机器人、130k 轨迹、700+任务；代码公开 | 单一公司生态、约 3 Hz、力控/安全与 OOD 受限 | 历史多任务基线；不替换高频控制器 |
| [PaLM-E](https://proceedings.mlr.press/v202/driess23a.html) | ICML 2023，PR | 将图像、机器人状态与文本交错注入 embodied multimodal LM | 多任务/多 embodiment 研究；模型不开放 | 规模大、端侧不现实；高层推理不等于精确动作 | 强调输入必须含状态、坐标系和时间，而非纯图像 |
| [VIMA](https://vimalabs.github.io/) / [VIMA-Bench](https://github.com/vimalabs/VimaBench) | ICML 2023，PR | 将文本、图像、视频等交错 multimodal prompt 映射为机器人动作，系统评估组合泛化 | benchmark、代码和训练轨迹公开 | 主要是仿真 tabletop；prompt 泛化不等于现实接触或安全 | 说明 typed multimodal prompt 的价值；仍只生成候选/phase |
| [RT-2](https://proceedings.mlr.press/v229/zitkovich23a.html) | CoRL 2023，PR | 将动作也编码成文本 token，把 web 视觉语言知识迁移到机器人 | 作者报告语义/OOD 增益；权重和训练数据闭源 | 语义泛化不等于 6D、接触或安全；难复现 | 用于类别/affordance advisory，不直接控制 |
| [Open X-Embodiment / RT-X](https://arxiv.org/abs/2310.08864) / [数据代码](https://github.com/google-deepmind/open_x_embodiment) | ICRA 2024，PR | 汇集多机构、多机器人数据并统一 RLDS/action schema | 论文报告 22 种机器人、527 skills、160,266 tasks；项目页版本数字更大 | 动作空间、频率、相机和数据质量高度异构 | 借鉴统一 episode/时间戳/动作 schema；SO-101 映射须显式验证 |
| [Octo](https://www.roboticsproceedings.org/rss20/p090.html) / [项目](https://octo-models.github.io/) | RSS 2024，PR | 在约 800k OXE 轨迹上预训练开放通用 policy，支持语言/目标图像和 diffusion head | 代码、权重和微调示例公开 | 仍需 embodiment 微调；作者 benchmark 非普适保证 | 可离线研究通用 policy adapter；每个动作块过 gate |
| [OpenVLA](https://proceedings.mlr.press/v270/kim25c.html) / [代码](https://github.com/VLA-RL/openvla) | CoRL 2024（PMLR 2025），PR | 7B VLA，以 DINOv2+SigLIP 和 Llama 系骨干学习 OXE 动作 token | 代码、权重与微调配方公开 | 7B 端侧成本高；窄任务专用 policy 常更强；数据/embodiment 敏感 | 适合离线/LoRA 对照，不宜作普通端侧实时控制器 |
| [RDT-1B](https://proceedings.iclr.cc/paper_files/paper/2025/file/49f80e4d2471ad4f2edf4f5f1ab62339-Paper-Conference.pdf) / [项目](https://rdt-robotics.github.io/rdt-robotics/) | ICLR 2025，PR | 约 1.2B diffusion foundation policy，统一双臂/多机器人动作表征 | 论文报告约 1M episodes、46 数据集和少样本适配 | 统一表示仍需每种 embodiment 校准；无形式安全保证 | 借鉴 action normalization，但关节/末端/夹爪边界必须 typed |
| [π0](https://www.roboticsproceedings.org/rss21/p010.html) / [openpi](https://github.com/Physical-Intelligence/openpi) | 首发 2024-10-31；RSS 2025，PR | VLM 上叠加 flow-matching action expert，生成连续动作块 | 代码和部分模型生态公开 | 完整训练数据/配方与跨机器人复现有限；时延和安全未解决 | 学习“语义慢层 + 连续动作头”，执行仍走 plan/gate |
| [π0.5](https://proceedings.mlr.press/v305/black25a.html) | 首发 2025-04-22；CoRL 2025，PR | 联合异构机器人数据、web 语义、高层子任务和低层动作，面向长时程开放环境 | openpi 支持模型；论文由公司团队在自有协议评估 | “open-world”只对报告分布成立；完整数据/独立复现不足 | 借鉴层次化 phase/skill 接口，不将开放世界输出当安全事实 |
| [OpenVLA-OFT](https://www.roboticsproceedings.org/rss21/p017.html) / [代码](https://github.com/moojink/openvla-oft) | 首发 2025-02-27；RSS 2025，PR | 并行解码、连续动作、action chunk、L1/微调配方提升 OpenVLA | LIBERO 与 ALOHA/真实平台作者实验；代码公开 | benchmark/embodiment 依赖；动作块仍有延迟和过期 | 比较通用 VLA 与小型 ACT/DP3，但统一通过 typed adapter |
| [FAST](https://www.roboticsproceedings.org/rss21/p012.html) | 首发 2025-01-16；RSS 2025，PR | DCT+BPE 压缩连续动作 chunk，降低自回归 token 长度 | tokenizer 公开；“压缩/训练加速”是作者协议结果 | 重建误差、动作边界、解码延迟和 stale chunk | 测 token 数、重建误差、P95、stale reject；不是 safety layer |
| [TinyVLA](https://tiny-vla.github.io/) / [代码](https://github.com/codhuck/tinyvla) | IEEE RA-L 2025，PR | 70M–1.4B 紧凑 VLM + diffusion decoder，减少大规模预训练依赖 | 代码公开；作者有仿真/真实评测 | 小模型不自动意味着 OOD 稳定或 CPU 实时 | 比 7B 更适合作为端侧候选；重点测资源/时效/拒绝 |
| [SmolVLA](https://arxiv.org/abs/2506.01844) / [模型](https://huggingface.co/lerobot/smolvla_base) | 首发 2025-06-02；截至截止日 PP | 约 450M、flow action expert、视觉 token 压缩与异步推理 | LeRobot 生态中代码/模型公开；SO-101 报告只覆盖单一 pick-place，且模型未用该 SO-101 数据预训练 | 未核验正式 venue；CPU/端到端延迟和恢复需独立测量 | 与 SO-101 硬件生态直接，但异步 chunk 必须完整、短时、可拒绝 |
| [GR00T N1](https://arxiv.org/abs/2503.14734) / [代码](https://github.com/NVIDIA/Isaac-GR00T) | 2025，TR/PP | System 2 VLM + System 1 diffusion Transformer，混合真实、人类和合成数据 | 代码栈和部分模型开放；N1 已迭代到后续产品版本 | 截止日未核验原始 N1 的同行评审；humanoid 与 SO-101 差异大 | 参考双系统设计，不复刻整套重型运行时 |
| [Gemini Robotics](https://arxiv.org/abs/2503.20020) / [技术报告](https://storage.googleapis.com/deepmind-media/gemini-robotics/gemini_robotics_report.pdf) | 首发 2025-03-25，TR/PP | Gemini VLA + Robotics-ER，强调空间、语义和跨 embodiment | 闭源；主要为公司/可信测试者结果 | 无公开权重/代码，厂商 benchmark 难独立复现 | 可作为高层目标/空间关系候选，执行留在本地安全栈 |
| [Gemini Robotics On-Device 2](https://deepmind.google/models/gemini-robotics/on-device/) / [模型卡](https://deepmind.google/models/model-cards/gemini-robotics-on-device-2) | 2026-07-30，Product | 面向本地设备与少样本新机器人适配；模型卡建议高低层安全分工 | 官方产品页和模型卡公开，未见可复现论文/权重 | 模型卡/公司评测，不是同行评审；OOD/高自由度局限仍在 | 佐证本地化趋势与外部 safety layer 必要性，不作为采用证明 |

**结论。** VLA 的收益集中在任务语义、跨任务复用和少样本适配；它们对 EdgeGrasp 的第一用途应是选择目标、affordance、phase 或短候选动作。SO-101 的坐标、关节范围、夹爪方向、观测时间和执行阶段必须由 typed adapter 显式绑定。

### 5.2 Diffusion/Flow Policy、模仿学习与离线 RL

| 工作 | 年份/状态 | 方法直觉与主要证据 | 局限 | EdgeGrasp 启示 |
|---|---|---|---|---|
| [DAgger](https://proceedings.mlr.press/v15/ross11a.html) | AISTATS 2011，PR | 在 learner 访问到的状态上反复查询专家，缓解纯 BC 的 covariate shift | 专家成本高；不能自动处理安全边界 | 在仿真专门采集目标丢失、阶段错位、near-failure 状态 |
| [CQL](https://proceedings.neurips.cc/paper/2020/hash/0d2b2061826a5df3221116a5085a6052-Abstract.html) | NeurIPS 2020，PR | 保守压低数据分布外动作的 Q 估计 | 奖励、覆盖和 Q 校准仍可错 | 仅作候选排序/风险 advisory |
| [IQL](https://openreview.net/forum?id=68n2s9ZJWF8) | ICLR 2022，PR | expectile value + advantage-weighted BC，避免显式查询未见动作 | 数据支持域外仍弱；无物理安全保证 | 可从安全日志学相对偏好，不替代 gate |
| [Decision Transformer](https://proceedings.neurips.cc/paper/2021/hash/7f489f642a0ddb10272b5c31057f0663-Abstract.html) | NeurIPS 2021，PR | 将 offline RL 重写成 return-conditioned 序列建模 | 期望回报和序列似然都不构成动力学/安全约束 | 用于理解“动作也是序列 token”，不建议作为首个抓取策略 |
| [RoboMimic / What Matters](https://proceedings.mlr.press/v164/mandlekar22a.html) | CoRL 2021，PR | 系统比较 BC/offline RL、数据质量和训练细节 | 结论依 benchmark；停止准则和示范质量高度敏感 | 借鉴数据 schema、质量分层和复现协议 |
| [ACT / ALOHA](https://roboticsproceedings.org/rss19/p016.html) | RSS 2023，PR | Action Chunking Transformer 与 temporal ensemble 降低长序列误差 | 双臂/demo 分布较窄；chunk 仍会过期 | 适合作为小模型 BC 基线；逐 chunk 绑定 phase/stamp |
| [Flow Matching](https://openreview.net/pdf?id=PqvMRDCJT9t) | ICLR 2023，PR | 学习概率路径上的向量场，以 ODE 生成连续样本；成为后续 flow action head 的基础 | 更少采样步不等于低端到端延迟或安全 | 为 π0/SmolVLA 等连续动作头提供理论背景 |
| [Diffusion Policy](https://roboticsproceedings.org/rss19/p026.html) | RSS 2023，PR | 对多峰连续动作做条件去噪，短滚动执行 | 多步采样、算力和延迟；无安全证明 | 与 ACT 做同数据、同 gate、同延迟预算比较 |
| [Q-Transformer](https://proceedings.mlr.press/v229/chebotar23a.html) | CoRL 2023，PR | 将动作维度离散成 token 并学习 Q，用混合人类/自主数据 | 离散化和 Q 仍可能产生错误乐观值 | feasibility score 只能参与排序 |
| [MimicGen](https://proceedings.mlr.press/v229/mandlekar23a.html) / [代码](https://github.com/NVlabs/mimicgen) | CoRL 2023，PR | 从少量人类源示范程序化组合大量仿真轨迹 | 合成覆盖不保证现实动力学 | 生成遮挡、延迟、位姿变化与 near-failure 轨迹 |
| [3D Diffusion Policy](https://www.roboticsproceedings.org/rss20/p067.html) / [代码](https://github.com/YanjieZe/3D-Diffusion-Policy) | RSS 2024，PR | 稀疏点云编码 + diffusion policy，显式利用 3D 几何 | 深度/外参误差、点云噪声和推理成本 | 比纯 RGB 更贴近 EdgeGrasp；先做离线/仿真 |
| [Consistency Policy](https://www.roboticsproceedings.org/rss20/p071.html) | RSS 2024，PR | 将 diffusion policy 蒸馏为更少采样步，降低推理延迟 | 蒸馏误差和 OOD 仍需测；速度不等于安全 | 在同一硬件比较质量—延迟—stale 的 Pareto 前沿 |
| [SERL](https://arxiv.org/abs/2401.16013) / [项目](https://serl-robot.github.io/) | ICRA 2024，PR/System | 以图像奖励、reset 和 off-policy RL 做样本高效真实机器人训练 | 需要真实交互，reward classifier 与 reset 仍有风险 | 当前文档阶段只作为未来路线，不建议半年先上真实 online RL |
| [Reactive Diffusion Policy](https://www.roboticsproceedings.org/rss21/p052.html) | RSS 2025，PR | 慢视觉计划 + 快触觉响应的双频闭环 | 触觉与硬件绑定，证据任务有限 | 支持未来 CLOSE/LIFT 快环，但先把时钟/接触 observer 做稳 |

**路线取舍。** 半年个人项目优先 BC/ACT 或 DP3，而不是 offline/online RL。RL 需要清晰奖励、失败覆盖和真实交互审计；在 EdgeGrasp 尚无物理成功基线时，先上 RL 很容易把 simulator/reward 偏差优化成“看似成功”。

### 5.3 World Model、视频-动作模型与合成“想象”数据

| 代表工作 | 年份/状态 | 方法与证据 | 局限 | EdgeGrasp 用法 |
|---|---|---|---|---|
| [World Models](https://arxiv.org/abs/1803.10122) | 2018，基础工作 | 在视觉潜变量中学习动力学并训练控制器 | toy/sim 任务；模型偏差大 | 提供“预测后果”概念，不是机器人执行证据 |
| [DayDreamer](https://proceedings.mlr.press/v205/wu23c.html) | CoRL 2022，PR | 将 Dreamer 类 latent world model 用于多种真实机器人 | 平台/任务窄，真实交互仍贵，无形式安全 | 说明 world-model RL 可上真实系统，也说明需强工程外壳 |
| [DreamerV3](https://www.nature.com/articles/s41586-025-08744-2) / [代码](https://github.com/danijar/dreamerv3) | Nature 2025，PR | 统一 latent dynamics、actor/critic imagination，在 150+ 多类任务用固定超参 | 强证据主要来自游戏/仿真；长时预测和真实接触安全未解决 | 借鉴不确定性/回放，不直接作为抓取控制器 |
| [TD-MPC2](https://openreview.net/forum?id=Oxh5CstDJU) / [代码](https://github.com/nicklashansen/tdmpc2) | ICLR 2024，PR | 在 latent dynamics 中做短 horizon MPC，覆盖 104 个连续控制任务 | model exploitation、在线数据和仿真偏差 | future cost 可作 advisory，真实执行仍需 plan/gate |
| [RoboDreamer](https://arxiv.org/abs/2404.12377) / [代码](https://github.com/rainbow979/robodreamer) | ICML 2024，PR | 语言分解 + 视频生成，组合未见机器人动作 | 生成视频逼真不等于动力学/接触真实 | 只用于离线合成和 failure hypothesis |
| [UWM](https://www.roboticsproceedings.org/rss21/p015.html) / [代码](https://github.com/WEIRDLabUW/unified-world-model) | RSS 2025，PR | 统一 action/video diffusion，以不同时间步承担 policy、forward/inverse dynamics | 视频—动作对齐和模型偏差仍是核心 | 产生候选未来状态，不直接发命令 |
| [DreamGen](https://proceedings.mlr.press/v305/jang25a.html) / [代码](https://github.com/nvidia/gr00t-dreams) | CoRL 2025，PR | 微调视频 world model，生成新视频，再用 latent/inverse dynamics 生成伪动作训练 policy | pseudo-action 误差、像素真实与接触物理脱节 | 适合扩增 near-failure/场景数据；必须保留真实 holdout |
| [V-JEPA 2](https://arxiv.org/abs/2506.09985) / [代码](https://github.com/facebookresearch/vjepa2) | 2025，PP | 从大规模视频学潜在预测表征，并用少量机器人数据做目标条件规划；论文含有限 Franka 零样本 pick/place 示范 | 未核验正式 venue；主要是表征/规划，不是 SO-101 低层策略 | 可离线评估 future representation，不外推至 SO-101 |
| [Cosmos](https://arxiv.org/abs/2501.03575) / [代码](https://github.com/NVIDIA/Cosmos) | 2025，TR/Product | 世界基础模型、视频 tokenizer、数据整理和合成平台 | 物理 fidelity 与动作精度不能由产品宣传推出 | 作为合成数据工具候选，不进入当前验收 |
| [Cosmos Policy](https://arxiv.org/abs/2601.16163) / [代码](https://github.com/NVlabs/cosmos-policy) | ICLR 2026，PR | 利用预训练世界模型表征并学习机器人 policy | 主要仿真/ALOHA 作者结果；端侧和跨机器人未充分证明 | 观察“world model → policy”趋势，不作为近期集成优先项 |
| [DreamDojo](https://arxiv.org/abs/2602.06949) / [代码](https://github.com/NVIDIA/DreamDojo) | 2026，PP；项目/作者页标注 ICML 2026，proceedings 未独立核验 | 从大规模 egocentric video 学 latent action world model，并开放部分代码/checkpoint | 作者帧率与跨 embodiment 结果未独立复现；视频预测仍非接触真值 | 作为 2026 世界模型规模化趋势跟踪项 |
| [DreamZero](https://arxiv.org/abs/2602.15922) / [代码](https://github.com/dreamzero0/dreamzero) | 2026，PP/TR | 联合视频与动作 diffusion，公开项目、代码和 checkpoint | 大模型算力重，venue 未确认，生成未来无 fail-closed 保证 | 仅用于离线 future/数据研究，不进入控制链 |

**核心分界。** 视频看起来合理，不等于接触、摩擦、关节力矩和时钟是正确的；world model 生成的 reward 或 future 也可能被 policy 利用。EdgeGrasp 可用它生成测试假设、近失败场景或候选轨迹，但不允许 imagined success 更新 <code>physics_grasp_verified</code>。

### 5.4 多机器人数据、通用与移动操作

| 工作 | 年份/状态 | 主要贡献与开放性 | 局限 | EdgeGrasp 启示 |
|---|---|---|---|---|
| [DROID](https://www.roboticsproceedings.org/rss20/p120.html) / [代码/数据](https://github.com/droid-dataset/droid) | RSS 2024，PR | 当前项目页口径约 76k trajectories、350h、564 scenes；论文版本 84 tasks，当前 release 86；采集工具与数据开放 | 主要 Franka+ZED；版本口径与数据分布需锁定 | 借鉴相机、标定、状态、动作、失败和采集元数据 |
| [CrossFormer](https://proceedings.mlr.press/v270/doshi25a.html) / [代码](https://github.com/rail-berkeley/crossformer) | 首发 2024-08-21；CoRL 2024（PMLR 2025），PR | 约 900k trajectories；论文版本 20 种 embodiment，当前项目页口径 30，覆盖单/双臂、移动、四足 | 版本差异、训练算力、数据比例和动作映射影响迁移 | 支持“通用 policy + 外部安全执行层” |
| [RoboMIND](https://www.roboticsproceedings.org/rss21/p152.html) / [项目](https://x-humanoid-robomind.github.io/) | RSS 2025，PR | 官方版约 107k trajectories、479 tasks、4 embodiments，并包含有原因的失败样本和数字孪生 | 早期/当前数据量口径不同；平台覆盖有限 | 统一 <code>stale_target</code>、<code>clock_mismatch</code>、<code>plan_invalid</code>、<code>grasp_lost</code> 失败 taxonomy |
| [AgiBot World](https://arxiv.org/abs/2503.06669) / [仓库](https://github.com/OpenDriveLab/AgiBot-World) | IROS 2025，PR | 官方 Beta release 口径超过 1M trajectories，含 100+ robots、移动/触觉/灵巧手；数据与代码公开 | 厂商生态、版本口径会变；CC BY-NC-SA 4.0 等非商用许可；独立复现有限 | 学数据协议，不追求个人项目规模 |
| [LeRobot](https://openreview.net/pdf?id=CiZMMAFQR3) / [代码](https://github.com/huggingface/lerobot) | ICLR 2026，PR/System | 统一低层电机 middleware、数据采集/存储/流式、训练、异步推理与低成本硬件生态 | 是基础设施而非通用能力证明；支持 SO-ARM101 不等于当前项目已采用 | 适合借鉴 episode schema、replay 和 SO-101 适配边界 |
| [Mobile ALOHA](https://mobile-aloha.github.io/) / [代码](https://github.com/MarkFzp/mobile-aloha) | CoRL 2024，PR | 低成本移动双臂遥操作与 BC，共训静态/移动数据 | 硬件和全身协调复杂，任务仍窄 | 固定基座四阶段先成为 skill contract，再谈移动化 |
| [OK-Robot](https://ok-robot.github.io/) | 2024，PP/System | 模块化导航、语言目标、抓取与放置；真实家庭系统试验 | 预印本/项目结果；总体成功率受模块串联系统性限制 | 说明模块化仍有竞争力；每模块必须独立计错 |
| [BiGym](https://proceedings.mlr.press/v270/chernyadev25a.html) | CoRL 2024，PR | 40 个移动双臂家庭仿真任务和人类示范 | 仿真 benchmark，不是现实泛化 | 可扩展四阶段为长时任务，但非半年优先 |
| [Mobi-π](https://proceedings.mlr.press/v305/yang25b.html) | CoRL 2025，PR | 选择移动底盘/视角，使操作 policy 留在训练分布内 | 结果依平台和 3DGS/视角模型 | 后续增加 viewpoint/base-pose 可见性与可达性前置条件 |

### 5.5 合成数据、仿真和 sim-real co-training

- [ManiSkill2](https://openreview.net/forum?id=b_CQDy9vrD1)（ICLR 2023，PR）提供多任务、多物体、多模态仿真基准；适合 deterministic replay 和扰动，但 simulator success 不是现实成功。
- [MimicGen](https://proceedings.mlr.press/v229/mandlekar23a.html)（CoRL 2023，PR）展示从少量源示范程序化扩展大量仿真轨迹；数据覆盖增加不等于动力学正确。
- [RoboCasa](https://www.roboticsproceedings.org/rss20/p050.html)（RSS 2024，PR）扩大厨房场景、物体和任务组合；其 LLM/生成资产仍需质量检查。
- [RoboTwin](https://openaccess.thecvf.com/content/CVPR2025/html/Mu_RoboTwin_Dual-Arm_Robot_Benchmark_with_Generative_Digital_Twins_CVPR_2025_paper.html)（CVPR 2025，PR）从图像/空间关系构造双臂 digital twin；生成几何、LLM 代码与接触均可能产生伪影。
- [RoboTwin 2.0](https://arxiv.org/abs/2506.18088)（2025 首发；[ICML 2026 官方下载页](https://icml.cc/Downloads/2026)与项目文档已列入会议，PR）扩展到更多对象、任务与 embodiment；所有提升仍是作者协议结果，不能外推为独立物理成功率。
- [Sim-and-Real Co-Training](https://co-training.github.io/)（RSS 2025，PR）与 [OT-based Domain Adaptation](https://ot-sim2real.github.io/)（NeurIPS 2025，PR）表明混合 sim/real 可能提高作者协议下的策略表现；相对提升不可作为通用比例。
- [RoboCasa365](https://proceedings.iclr.cc/paper_files/paper/2026/hash/a05003fdb1e9562ab0c0a9719ea4de10-Abstract-Conference.html)（ICLR 2026，PR）进一步扩大组合任务与 synthetic/human 数据；适合借鉴原子技能/阶段标签，不适合个人项目追规模。

对 EdgeGrasp，合成数据的优先用途依次是：**时间/遮挡/噪声/目标运动的可重复 stress test → near-failure 覆盖 → 少量策略训练 → 有硬件后配对校准**。不能倒过来先追求大模型 success rate。

### 5.6 触觉、多模态与接触闭环

| 工作 | 年份/状态 | 贡献 | 局限 | EdgeGrasp 增量 |
|---|---|---|---|---|
| [Sparsh](https://sparsh-ssl.github.io/) | CoRL 2024，PR | 多触觉传感器自监督表征与 TacBench；代码/数据/checkpoint 公开 | 视觉触觉的传感器、光照和标定差异显著 | 先把 contact/slip 作为 CLOSE/LIFT observer，而非端到端控制 |
| [AnyTouch](https://gewu-lab.github.io/AnyTouch/) | ICLR 2025，PR | 统一多种触觉传感器，并与视觉/文本对齐；数据与代码公开 | 真实任务规模有限，跨传感器校准仍难 | 参考 typed tactile observation 与 calibration metadata |
| [TACTO](https://arxiv.org/abs/2012.08456) / [代码](https://github.com/facebookresearch/tacto) | IEEE RA-L 2022，PR/System | 快速视觉触觉仿真，便于生成触觉图像和策略训练数据 | 不完整模拟形变、摩擦与真实传感器漂移；仿真触觉不是现实接触 | 用于离线接口和 fault test，不更新物理 outcome |
| [3D-ViTac](https://binghao-huang.github.io/3D-ViTac/) | CoRL 2024，PR | 将视觉和触觉统一到 3D 表示，面向接触丰富操作 | 传感器/机器人特定，3D 对齐和同步仍是瓶颈 | 为未来 wrist/contact observation 的坐标与时间 schema 提供参考 |
| [Hearing Touch](https://collaborate.princeton.edu/en/publications/hearing-touch-audio-visual-pretraining-for-contact-rich-manipulat/) | ICRA 2024，PR | 接触麦克风+音视预训练辅助接触密集任务 | 易受环境/电机振动；不适合成为唯一停止依据 | 可作 close/lift 辅助事件，不单独判定抓取 |
| [Reactive Diffusion Policy](https://www.roboticsproceedings.org/rss21/p052.html) | RSS 2025，PR | 慢视觉 diffusion + 快触觉 feedback 的双频策略 | 任务/传感器有限，硬件同步复杂 | 长期参考双时间尺度；当前优先独立物理 observer |
| [AnyTouch 2](https://proceedings.iclr.cc/paper_files/paper/2026/hash/073c8584ef86bee26fe9d639ec648e28-Abstract-Conference.html) | ICLR 2026，PR | ToucHD 动态触觉数据与统一物体级/力感知表征；代码、数据和模型公开 | 跨传感器、力标定和实际任务覆盖仍需复核 | 可作为未来动态接触表征，不纳入当前无硬件验收 |

### 5.7 具身推理与学习型安全监测

| 工作 | 年份/状态 | 核心贡献 | 证据局限 | EdgeGrasp 关系 |
|---|---|---|---|---|
| [ECoT](https://proceedings.mlr.press/v270/zawalski25a.html) | CoRL 2024/正式 PMLR 2025，PR | VLA 在动作前生成子任务、视觉框、末端位置和 embodied chain-of-thought | 推理会错且更慢；解释不是安全证明 | 接收其几何/阶段假设，不接收自由文本命令 |
| [SpatialVLA](https://www.roboticsproceedings.org/rss21/p011.html) | RSS 2025，PR | 以 Ego3D position encoding 和 adaptive action grids 强化 3D 空间动作表示 | 仍受深度、外参与坐标误差影响 | 可研究 3D action representation；必须绑定 frame/stamp |
| [AHA](https://proceedings.iclr.cc/paper_files/paper/2025/hash/70a06501001e1820fd1eb9ee821302d2-Abstract-Conference.html) / [代码](https://github.com/NVlabs/AHA) | ICLR 2025，PR | 从扰动的成功轨迹学习解释机器人失败 | 主要仿真失败类别；语言解释可能错误 | 用于生成 failure reason/advisory，不替代传感器检查 |
| [SAFE](https://proceedings.neurips.cc/paper_files/paper/2025/hash/392d0d05e2f514063e6ce6f8b370834c-Abstract-Conference.html) | NeurIPS 2025，PR | 从 VLA 内部特征学失败概率，并以 conformal prediction 管理检测精度/提前量 | 校准依赖数据，OOD 可漏报，监测也有延迟 | 放在 gate 前作附加 monitor；monitor 异常时仍 fail closed |
| [SafeVLA](https://proceedings.neurips.cc/paper_files/paper/2025/hash/e185c7be603426028c32ae1003a59d78-Abstract-Conference.html) | NeurIPS 2025 Main Conference，PR | 在模拟长时导航/操作中主动生成 hazard 并做 constrained RL | curated simulation risk 与真实力/扭矩风险不同 | 借鉴 hazard matrix，不宣称现有 gate 已是完整 SafeRL/CBF |

学习型 failure monitor 的合理位置是传统 gate 的**上游附加信号**，不是替代品。它可以更早发现失败倾向，却不能证明所有未报警动作安全。

---

## 6. 2025–2026 新论文与公开系统状态审计

下表专门防止把首发日、修订日、会议接收、代码发布日期和产品公告混为一谈。

| 工作 | 首次公开 / 截止日状态 | 开放性核对 | 需保留的 caveat |
|---|---|---|---|
| [FAST](https://arxiv.org/abs/2501.09747) | 2025-01-16；[RSS 2025](https://www.roboticsproceedings.org/rss21/p012.html)，PR | tokenizer 公开 | 作者的压缩/训练加速不是端到端控制提升 |
| [OpenVLA-OFT](https://arxiv.org/abs/2502.19645) | 2025-02-27；[RSS 2025](https://www.roboticsproceedings.org/rss21/p017.html)，PR | [代码](https://github.com/moojink/openvla-oft) 公开 | LIBERO/ALOHA 结果不能写成 SO-101 物理抓取 |
| [π0](https://arxiv.org/abs/2410.24164) | **首发 2024-10-31**；RSS 2025，PR | openpi 代码/部分模型公开 | 不应把 RSS 年份当首发年份；完整训练资产未全开 |
| [π0.5](https://arxiv.org/abs/2504.16054) | 2025-04-22；CoRL 2025，PR | openpi 支持模型 | “open-world”需限定作者任务/环境分布 |
| [SmolVLA](https://arxiv.org/abs/2506.01844) | 2025-06-02；截至 2026-08-27 未核验正式 venue，PP | 约 450M 模型与 LeRobot 代码生态公开 | 异步/CPU 相关结论必须按指定硬件实测 |
| [GR00T N1](https://arxiv.org/abs/2503.14734) | 2025-03-18；TR/PP | Isaac-GR00T 代码栈公开；代码、权重、数据许可需分别看 | 后续 N1.x 产品更新不能倒推原论文已同行评审 |
| [Gemini Robotics](https://arxiv.org/abs/2503.20020) | 2025-03-25；TR/PP | 无公开权重/代码 | 博客发布时间不能替代论文首发；公司评测非独立复现 |
| [Figure Helix](https://www.figure.ai/news/helix) | 2025-02-20；公司 Product/TR | 未见公开论文、代码或权重 | 7–9 Hz / 200 Hz 等为公司披露，仅作双时间尺度参照 |
| [AgiBot World](https://arxiv.org/abs/2503.06669) | 2025-03-09；IROS 2025，PR | 代码/数据公开，Beta release 采用 CC BY-NC-SA 4.0 等许可 | Alpha/Beta 数据量和模型版本不同，必须记录 release |
| [DreamGen](https://proceedings.mlr.press/v305/jang25a.html) | 2025 首发；CoRL 2025，PR | [代码](https://github.com/nvidia/gr00t-dreams) 公开 | 伪动作和视频世界模型误差不能视作现实泛化 |
| [V-JEPA 2](https://arxiv.org/abs/2506.09985) | 2025-06-11；截止日未核验 venue，PP | [代码](https://github.com/facebookresearch/vjepa2) 公开 | 是表征/世界模型路线，不是低层抓取策略 |
| [GraspGen](https://arxiv.org/abs/2507.13097) | 2025-07-17；ICRA 2026，PR | 代码/数据工具公开 | 论文 53M+ 与当前仓库 57M+ 是版本口径差异 |
| [RoboTwin 2.0](https://arxiv.org/abs/2506.18088) | 2025-06-22；ICML 2026，PR | 代码/平台公开；[ICML 官方下载页](https://icml.cc/Downloads/2026)已列入 | 生成资产、接触物理和作者协议结果仍需现实 holdout |
| [AnyTouch 2](https://proceedings.iclr.cc/paper_files/paper/2026/hash/073c8584ef86bee26fe9d639ec648e28-Abstract-Conference.html) | 2026-02；ICLR 2026，PR | 代码、ToucHD 数据和模型公开 | 公开不等于已验证跨传感器标定或 SO-101 接触 |
| [Cosmos Policy](https://arxiv.org/abs/2601.16163) | 2026-01-22；ICLR 2026，PR | [代码](https://github.com/NVlabs/cosmos-policy) 公开 | 仿真/ALOHA 指标不是 EdgeGrasp 实体证据 |
| [LeRobot](https://openreview.net/pdf?id=CiZMMAFQR3) | 2026-02-26；ICLR 2026，PR | [代码](https://github.com/huggingface/lerobot) 公开 | 支持 SO-ARM101 不等于 EdgeGrasp 已迁移或已抓取 |
| [GraspGen-X](https://arxiv.org/abs/2606.00998) | 2026-05-31；CVPR 2026，PR | 代码/checkpoint 公开；完整训练数据待发布；代码与模型许可证不同 | 项目页 395M/350M sampled 与仓库 >2B computed 是不同口径；跨夹爪不等于 SO-101 适配 |
| [Grasp-MPC](https://research.nvidia.com/labs/lpr/publication/yamada2026graspmpc/) | 2025-09-07 首发；ICRA 2026，PR | 项目页代码仍 Coming soon | 作者报告的 sim/real 相对改进依指定协议 |
| [SpaHybGen](https://www.nature.com/articles/s42256-026-01292-y) | 2026-08-12 version of record；Nature Machine Intelligence，PR | [代码](https://github.com/wangzivector/SpaHybGen) 与模型公开 | 多指/多手证据不能直接迁移两指 SO-101；细节需全文复核 |
| [Jetson-PI](https://arxiv.org/abs/2607.12659) | 2026-07-14；PP | [代码](https://github.com/PKU-SEC-Lab/Jetson-PI) 公开 | 截止日未核验 peer-reviewed venue；Jetson/LIBERO 指标为作者协议 |
| [VLAff](https://arxiv.org/abs/2608.05215) | 2026-08-05；PP（arXiv comments 注明 accepted IROS 2026） | 代码/数据与正式 proceedings 未独立核验 | 非常新，所有规模与能力均为预印本自报 |

常见误判规则：

1. arXiv 首发、修订、会议接收和 proceedings 发布是不同字段。
2. 公司博客/产品发布日期不能替代论文日期。
3. “代码开源”不推出权重、训练数据、配方和商业许可也开放。
4. 20 Hz、200 Hz 常是某个子模块速率，不是 sensor → inference → plan → gate → controller 的端到端频率。
5. zero-shot、generalist、open-world 只在论文指定分布与协议内成立。
6. SOTA/相对提升不能跨硬件、任务、分母和基线直接排名。

---

## 7. 综合吸收：路线关系、关键分歧与证据强弱

### 7.1 六组关键分歧

#### 分歧一：端到端泛化，还是显式几何与模块化安全

- VLA/基础模型的论点是：共享视觉语言先验和多机器人数据可减少每个任务从零训练。
- 几何/模块化路线的论点是：6D pose、手眼标定、碰撞、关节限位、目标时间和夹爪接触需要可解释接口。
- 在当前 EdgeGrasp 的数据规模、硬安全边界和可验证性约束下，本综述的工程判断更倾向于**组合式系统**：模型提供语义和候选，显式模块提供坐标/规划，runtime assurance 提供拒绝/停止。闭源通用模型尚未给出足以让本项目移除这些层的公开证据。

#### 分歧二：动作 token、自回归 chunk，还是 diffusion/flow

- 自回归 token 容易与 LLM/VLM 统一；FAST、BEAST、speculative decoding 等努力减少序列长度和延迟。
- Diffusion/flow 更自然地表达连续、多峰动作；π0、RDT、DP3 等代表这条路线。
- 两者都可能产生过期动作。EdgeGrasp 应比较的不是论文平均 success，而是**端到端 P95/P99、完整 chunk 可接受率、stale 拒绝率、切换不连续和 stop latency**。

#### 分歧三：开放环 action chunk，还是持续闭环

- action chunk 降低逐步误差和推理开销，但观测变化后整段动作可能失效。
- 视觉伺服和 receding horizon 更能响应动态目标，却增加高频控制权、时钟同步和稳定性难题。
- 可行折中是短 chunk + 明确有效期 + 每段重新观测；final approach 可使用严格限速/限时的 servo sandbox。

#### 分歧四：更多真实数据，还是更强仿真/合成

- OXE、DROID、RoboMIND、AgiBot 说明真实多机器人数据能扩大覆盖，但成本、数据质量与 action schema 异构严重。
- MimicGen、RoboCasa、RoboTwin、world model 能低成本扩增，但几何/接触/伪动作误差会被策略学习。
- 更可信的方向是**少量真实校准 + 大量可解释合成 + 真实 holdout + paired sim-real 分析**，而不是单纯堆一种数据。

#### 分歧五：行为克隆，还是 offline/online RL

- BC/ACT/DP 在示范分布内通常更容易复现和调试。
- offline RL 能利用失败与次优数据，但 Q 值、奖励、覆盖和保守性难审计。
- online RL 可能提高特定任务表现，却需要真实探索和安全 reset。
- 对当前 EdgeGrasp，应先有稳定物理 outcome 与失败 taxonomy，再考虑 RL；否则奖励模型很可能学习 simulator loophole。

#### 分歧六：高平均成功率，还是可验证的尾部风险

- 多数论文优化平均 task success；安全系统更关心低频但严重的目标错配、时钟回退、晚到结果、碰撞、滑落和停止失败。
- 学习型 failure detector 可改善提前量，却仍受校准与 OOD 限制。
- EdgeGrasp 的差异化价值应是把平均性能与**拒绝、停止、失效、恢复和物理 outcome**同时报告。

### 7.2 当前瓶颈

| 瓶颈 | 为什么仍未解决 | 可测量信号 |
|---|---|---|
| 3D/空间 grounding | VLM 语义强，但精确深度、对称、遮挡和外参仍会错 | pose median/P95、BOP 指标、frame/epoch 错误 |
| 接触与灵巧性 | 视觉难观察摩擦、力闭合、滑移和材料 | 双侧/全 pad 接触、lift、retention、slip、force/effort |
| 时间与延迟尾部 | 异步感知、规划和动作 chunk 跨不同频率与时钟 | capture-to-send P50/P95/P99、target age、jitter、drop |
| 跨 embodiment | action space、夹爪、视角、频率和安全限制不统一 | adapter rejection、关节/夹爪映射误差、校准漂移 |
| 数据质量 | 成功轨迹多、失败原因少；不同数据集元数据不一致 | failure taxonomy 覆盖、重复/缺帧、时间戳完整度 |
| Sim-to-real | simulator 视觉/动力学/接触与现实不同 | paired rank correlation、每个域的 failure distribution |
| 长时程错误累积 | 高层误解、低层小误差、恢复失败串联 | phase-wise success、首次失败阶段、恢复次数 |
| 端侧资源 | 大模型显存、功耗、热、抖动与网络依赖 | RAM/VRAM、功耗、温度、throttling、offline availability |
| 安全证明范围 | 真实系统包含感知、middleware、OS、控制器和硬件 | assumption ledger、fault injection、stop acknowledgement |
| 评测可比性 | 成功定义、对象、试验数、reset、人工介入不同 | 固定 protocol、分母、区间、配置/模型 hash |

### 7.3 证据强弱判断

**相对强：**

- 有正式同行评审版本；
- 公开清晰 protocol、分母、真实机器人重复试验和失败；
- 代码/数据/配置可获得，且第三方可在相似条件复现；
- 将 perception、control、physical outcome 分开。

**中等：**

- 正式论文但只有仿真或少量真实任务；
- 作者项目页/模型卡披露完整限制；
- 代码可用但数据、权重或训练配方不全。

**较弱：**

- 新 preprint、公司产品公告或只给精选视频；
- 只报告相对提升、SOTA、Hz 或总成功率；
- 分母、失败定义、人工干预、reset、硬件和延迟链不清；
- 将仿真/analytic grasp 指标直接解释为物理成功。

按这一尺度，EdgeGrasp 目前对 typed protocol、新鲜度、错误相关性和 fail-closed 的证据较强；对仿真物理抓取的证据是**明确的负结果**；对真实硬件、相机标定、端侧模型和实际停止则尚无证据。

---

## 8. 未来 2–3 年（2026 下半年至 2029）的可能发展

以下均是基于公开路线的**推断**，不是已发生事实。

1. **双时间尺度成为主流。** 语义/VLM/World Model 低频提出目标和计划；小型 flow/diffusion、servo 或触觉控制高频响应；外部 shield/gate 处理边界和停止。
2. **端侧小模型与异步 chunking 加速。** TinyVLA、SmolVLA、on-device 产品和 Jetson 优化说明本地化需求强；真正竞争点会从“能跑”转向 P99、热稳定、断网、观测年龄与停止响应。
3. **动作表示继续从离散 token 向连续/混合表示演进。** FAST 等压缩自回归动作，flow/diffusion 负责连续性，未来很可能混合语义 token、空间 token 与连续 motor head。
4. **跨 embodiment 不再只靠共享权重，而会显式建模形态。** gripper geometry、kinematic family、camera topology、frequency 和 safety envelope 将成为条件，GraspGen-X 类路线是早期信号。
5. **数据竞争转向“失败、干预、触觉和时间元数据”。** 成功视频已相对充足，真正稀缺的是可解释失败、恢复、接触、标定、延迟和人类介入。
6. **sim/real 混训会增加，但 paired validation 变成硬要求。** 生成视频和 digital twin 继续扩大；如果没有现实 holdout 和 simulator mismatch 分析，结果将越来越难信。
7. **世界模型更像数据/评估层，而非单独替代控制器。** 短期更现实的用途是生成场景、预测候选后果、发现失败和辅助规划；接触级真值仍需现实/高保真观测。
8. **学习型 safety monitor 增长，但不会取代传统安全层。** conformal failure monitor、CBF/shield 和 hazard-generation 可提高覆盖，仍需 fail-closed 的确定性检查与硬件停止。
9. **评测会从单一成功率向系统级 scorecard 转移。** 端到端延迟、恢复、干预、能耗、分阶段失败、真实保持和安全成本会逐步进入公开基准。

对 EdgeGrasp，这意味着现在投资 typed metadata、可回放 failure taxonomy 和独立 outcome observer，不是“绕开 AI”，而是在为未来模型提供可替换、可比较、可拒绝的落点。

---

## 9. EdgeGrasp 项目增量地图

### 9.1 优先级

| 优先级 | 项目增量 | 研究问题 | 验收证据 | 明确不声称 |
|---|---|---|---|---|
| P0 | 静态 RGB-D/tag/已知 CAD 定位与 replay | 坐标、外参、时间戳是否可靠 | 留出集 pose median/P95；30+ replay；frame/time failure | 未做硬件时不声称真实标定 |
| P0 | 当前仿真物理负结果闭环 | 为什么 3.711 mm 后失去接触 | 固定 config/seed 的 contact/lift/retention 分解 | 四阶段完成不叫抓取成功 |
| P1 | 学习型 grasp/pose typed adapter | 模型候选能否安全进入现有链 | model digest、pose/width/score/uncertainty、reject reason | 模型 score 不叫 physical success |
| P1 | 延迟注入与执行时目标预测 | 动态目标下何时应预测、重规划或停止 | 0/20/40 mm/s；无补偿/常速度/停止对照；P95/P99 | 仿真结果不叫动态实抓 |
| P2 | ONNX/TensorRT 端侧优化 | 质量—延迟—资源折中 | 精度、P50/P95/P99、VRAM/RAM、温度、长稳、drop | 桌面结果不替代 Jetson |
| P2 | 短时 visual-servo sandbox | final approach 是否受益于闭环 | 小范围限速、超时、collision/singularity、ownership 测试 | Servo 内置检查不叫完整安全认证 |
| P3 | ACT/DP3/SmolVLA 仿真对照 | 通用/策略模型是否值得复杂度 | 同数据、同扰动、同 gate 的 success/reject/stale/resource | policy 平均成功率不覆盖 tail risk |
| P3 | 学习型 failure monitor | 是否更早识别阶段失败 | calibration、precision/recall、lead time、OOD/fail behavior | monitor 不替代 gate |
| P4 | 触觉/音频辅助 | close/lift 的接触和滑移是否更可观测 | 传感器校准、同步、slip/retention 对照 | 单一分类器不置 physics true |
| P4 | paired sim-real | 仿真排序是否能预测现实 | 同对象/配置/协议的 paired matrix | 没有硬件时不声称 sim-to-real |
| P5 | VLA 高层 advisory | 语义目标/phase 是否有增益 | 目标选择、阶段错误、解释/延迟、人工审核 | VLA 不直接生成控制器命令 |

### 9.2 建议的 typed model record

    source_model / version / weight_digest
    observation_frame / robot_frame
    capture_timestamp / receive_timestamp / target_timestamp
    clock_domain / epoch / target_id
    phase = approach | descend | close_gripper | lift
    target_pose | grasp_pose | joint_action
    gripper_width / gripper_command
    confidence / covariance / horizon
    inference_latency / preprocessing_digest

所有动作或轨迹块必须：

1. 检查维度、NaN、frame、target ID、时钟域、epoch 与时间新鲜度；
2. 检查 phase 是否允许该动作；
3. 检查 workspace、关节、速度、加速度和夹爪边界；
4. 经过 tf2、IK、MoveIt plan-only、trajectory validation 和现有 typed gate；
5. 只提交完整、短时有效、可相关的动作块；
6. 时钟异常、目标过期、推理超时、模型/phase 错配或结果晚到时 fail closed；
7. 将 controller terminal 与 contact/lift/retention outcome 分开。

### 9.3 建议的失败 taxonomy

| 层 | 示例 |
|---|---|
| Sensor | frame drop、depth invalid、intrinsics/extrinsics mismatch、clock jump |
| Perception | no target、wrong ID、pose OOD、covariance high、track lost |
| Temporal | stale、future、epoch mismatch、inference overrun、chunk expired |
| Planning | tf2 fail、IK fail、collision、joint bound、trajectory invalid |
| Protocol | wrong phase、digest/correlation mismatch、late/cancel/result race |
| Controller | rejected、tolerance violation、timeout、cancel unacknowledged |
| Contact | no contact、one-pad only、contact too brief、slip |
| Outcome | lift below 20 mm、retention false、object drift、drop |
| Recovery | fallback unavailable、stop not acknowledged、restart/epoch failure |

---

## 10. 推荐阅读顺序

### 第一优先：直接支持当前项目（先读）

1. 当前 [项目 README](../README.md)、[验证报告](validation-report.md) 与 [Candidate011 观察](observations/2026-08-27-candidate011-q0p40-runtime.md)：先掌握项目真正证明了什么。
2. [MoveIt PlanningOptions](https://docs.ros.org/en/ros2_packages/rolling/api/moveit_msgs/msg/PlanningOptions.html)、[MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)、[JTC Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)：理解规划、伺服、控制器 terminal 与物理停止的边界。
3. [Dex-Net 2.0](https://roboticsproceedings.org/rss13/p58.html)、[GraspNet](https://openaccess.thecvf.com/content_CVPR_2020/papers/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.pdf)、[Contact-GraspNet](https://arxiv.org/abs/2103.14127)：学习 analytic grasp、数据驱动候选和真实 outcome 的区别。
4. [FoundationPose](https://openaccess.thecvf.com/content/CVPR2024/html/Wen_FoundationPose_Unified_6D_Pose_Estimation_and_Tracking_of_Novel_Objects_CVPR_2024_paper.html)、[BOP tasks](https://bop.felk.cvut.cz/tasks/)：学习 6D pose、对称、遮挡和独立评测。
5. [Closing the Loop](https://www.roboticsproceedings.org/rss14/p21.html)、[Dynamic Grasp Planning](https://link.springer.com/article/10.1007/s10514-018-9799-1)、[Real-Time Execution of Action Chunking Flow Policies](https://arxiv.org/abs/2506.07339)：把动态抓取理解成反馈与时间问题。
6. [Simplex](https://experts.illinois.edu/en/publications/the-simplex-architecture-for-safe-on-line-control-system-upgrades/)、[SOTER](https://arxiv.org/abs/1808.07921)、[VerifAI](https://doi.org/10.1007/978-3-030-25540-4_25)：理解不可信高性能层和独立安全层。

### 第二优先：策略与通用模型（建立谱系）

7. [ACT](https://roboticsproceedings.org/rss19/p016.html)、[Diffusion Policy](https://roboticsproceedings.org/rss19/p026.html)、[DP3](https://www.roboticsproceedings.org/rss20/p067.html)：先理解小型策略和连续动作。
8. [RT-1](https://roboticsproceedings.org/rss19/p025.html)、[RT-2](https://proceedings.mlr.press/v229/zitkovich23a.html)、[PaLM-E](https://proceedings.mlr.press/v202/driess23a.html)：理解 VLA 的起点与语义价值。
9. [OXE/RT-X](https://arxiv.org/abs/2310.08864)、[Octo](https://www.roboticsproceedings.org/rss20/p090.html)、[OpenVLA](https://proceedings.mlr.press/v270/kim25c.html)：理解跨机器人数据与开放模型。
10. [RDT-1B](https://proceedings.iclr.cc/paper_files/paper/2025/file/49f80e4d2471ad4f2edf4f5f1ab62339-Paper-Conference.pdf)、[π0](https://www.roboticsproceedings.org/rss21/p010.html)、[FAST](https://www.roboticsproceedings.org/rss21/p012.html)、[SmolVLA](https://arxiv.org/abs/2506.01844)：理解 diffusion/flow、token 压缩和端侧化。

### 第三优先：扩展路线（按项目需要读）

11. [MimicGen](https://proceedings.mlr.press/v229/mandlekar23a.html)、[DROID](https://arxiv.org/abs/2403.12945)、[RoboMIND](https://www.roboticsproceedings.org/rss21/p152.html)：数据协议、失败与合成扩增。
12. [DayDreamer](https://proceedings.mlr.press/v205/wu23c.html)、[TD-MPC2](https://openreview.net/forum?id=Oxh5CstDJU)、[UWM](https://www.roboticsproceedings.org/rss21/p015.html)、[DreamGen](https://proceedings.mlr.press/v305/jang25a.html)：World Model 的能力与模型偏差。
13. [Sparsh](https://sparsh-ssl.github.io/)、[AnyTouch](https://gewu-lab.github.io/AnyTouch/)、[Reactive Diffusion Policy](https://www.roboticsproceedings.org/rss21/p052.html)：触觉和快慢闭环。
14. [SAFE](https://proceedings.neurips.cc/paper_files/paper/2025/hash/392d0d05e2f514063e6ce6f8b370834c-Abstract-Conference.html)、[SafeVLA](https://proceedings.neurips.cc/paper_files/paper/2025/hash/e185c7be603426028c32ae1003a59d78-Abstract-Conference.html)：学习型失败监测的收益与边界。

### 12 周阅读节奏

| 周 | 主题 | 产出 |
|---|---|---|
| 1 | 项目、MoveIt、ros2_control、安全语义 | 一页系统边界图 + 术语表 |
| 2 | Dex-Net/GraspNet/Contact-GraspNet | 候选生成—物理 outcome 对照表 |
| 3 | 6D pose/FoundationPose/BOP | pose error、对称、tracking failure 指标表 |
| 4 | 闭环/动态/延迟 | 完整时间链与延迟预算 |
| 5 | Sim-to-real/评测 | 固定 protocol 与 failure taxonomy |
| 6 | ACT/DP/DP3 | action chunk 接口与过期风险 |
| 7 | RT-1/RT-2/PaLM-E | VLA 语义与低层动作边界 |
| 8 | OXE/Octo/OpenVLA | 数据 schema 与 embodiment gap |
| 9 | RDT/π0/FAST/SmolVLA | 端侧质量—延迟—资源比较方案 |
| 10 | World Model | imagined future 与 reality check 清单 |
| 11 | 触觉/SAFE/SOTER | monitor、gate、hardware stop 信任边界 |
| 12 | 2025–2026 状态复核 | 只保留可核验来源和未验证项 |

---

## 11. 面向求职者的方向选择

### 11.1 现有能力与缺口

假设候选人是 2021 年计算机本科、电视中间件/嵌入式软件背景，可准备半年：

**可迁移优势：**

- C/C++、Linux、设备/中间件接口、复杂协议和兼容性；
- 日志、问题定位、长稳、资源约束、版本/交付与测试意识；
- 更容易理解 ROS 2/DDS、时间戳、QoS、watchdog、硬件抽象和端侧性能。

**需补证据：**

- Python/PyTorch 与一个完整模型训练/评测闭环；
- 3D 几何、相机模型、tf2、外参与不确定性；
- ROS 2/MoveIt/ros2_control 的实际接口理解；
- ONNX/TensorRT 或同类部署优化的量化结果；
- 机器人 physical outcome、安全边界和 sim-to-real 证据分层。

### 11.2 2026-08-27 公开岗位信号

岗位页面会变化，以下只用于技能映射，不代表仍有名额或个人资格：

- [穹彻智能 Noematrix 招聘](https://www.noematrix.ai/join-us)：上海岗位信号；机器人算法/AI 岗同时强调 C++/Python、通信架构、运动规划、仿真、检测/分割/跟踪、3D pose、压缩、VLM 与部署；RL/VLA 岗还列出 ACT、Diffusion Policy、OpenVLA、π0 与 sim-to-real 延迟噪声。
- [宇树招聘](https://www.unitree.com/cn/position/)：杭州岗位信号；包括 Jetson Orin/Thor/Ascend 部署、低延迟、数据闭环、安全停止/恢复、长时间测试、量化/蒸馏与嵌入式 Linux/C++。
- [DJI 热门岗位](https://careers.dji.com/zh-CN/campus/hot-jobs?source=campus_hotjobs)：出现 VLM 推理部署、3D/空间智能、世界模型、边缘 AI、NPU/DSP/编译器和 OS/中间件；页面为校招趋势信号，不代表 2021 年毕业者资格。
- [Field AI Robotics AI Engineer](https://jobs.lever.co/field-ai/f326c7a0-84ee-4f91-aa16-80281203e8d3) 与 [Embedded Compute Engineer](https://jobs.lever.co/field-ai/7a0d12c7-957b-42f6-a9c9-9605fab6d2a4)：新加坡 onsite 全职岗位信号；同时强调 Python/C++、ML/ROS 2、感知/融合，以及 Jetson、Linux/Yocto、总线、热/功耗、延迟、watchdog 和诊断。

没有公开数据能支持某一方向的个人“录用概率”百分比。合理的相对分层是：

| 目标岗位 | 相对匹配度与半年可验证性（启发式，非录用概率） | 主要补强 | 项目证据 |
|---|---|---|---|
| 机器人中间件/嵌入式 AI 系统/现场集成 | 较高 | ROS 2/DDS、传感器、watchdog、部署 | EdgeGrasp typed gate、时间链、故障注入、长稳 |
| 端侧感知部署/3D 定位 | 中等 | 相机几何、PyTorch、ONNX/TensorRT、量化 | RGB-D pose + 精度/延迟/资源 scorecard |
| 操作/抓取算法工程 | 中低 | 3D CV、IL/DP、实验设计 | 候选模型 + dynamic latency + 分阶段 outcome |
| VLA/World Model/纯 RL 研究 | 较低 | 大规模训练、论文/研究经历、深度数学与算力 | 半年个人项目难形成同等规模证据 |

建议投递叙事：**“嵌入式可靠性与端侧部署工程师，正在把 AI 感知安全地接入 ROS 2 机器人执行链”**，而不是“接入过某个 VLA”。

---

## 12. 六个月 EdgeGrasp 项目映射

> 这是后续建议，不是本次已执行工作。表中 30+ replay、0/20/40 mm/s、P95/P99 等是建议的实验协议/示例目标，不是现状、行业门槛或已取得结果。若没有真实硬件，所有产出必须明确标注仿真/回放。

| 月份 | 主题 | 核心工作 | 可量化验收 | 求职信号 |
|---|---|---|---|---|
| 第 1 月 | 可重复感知与时间基线 | 相机健康/合成输入、capture/receive 时钟、MCAP/离线 replay、统一 config/hash | 30+ replay；帧率/丢帧；sensor→decision P50/P95；CPU/RAM；相同输入结果一致 | ROS 2、日志、时钟、测试、系统性 |
| 第 2 月 | 3D 定位与 tf2 | tag/已知 CAD 经典基线；intrinsics/extrinsics；留出集；frame/epoch failure | 3D/姿态 median/P95；tf2 failure；校准版本；outlier taxonomy | 相机几何、3D、传感器集成 |
| 第 3 月 | 学习型感知与端侧部署 | 加一个 detector/pose/grasp model；与经典基线同协议；ONNX/TensorRT（仅在可用硬件） | 精度/AP 或 pose 指标；P50/P95/P99；VRAM/RAM/温度；长稳；model digest | PyTorch、部署、性能工程 |
| 第 4 月 | 动态目标与延迟补偿 | 0/20/40 mm/s、掉帧/抖动；无补偿 vs 常速度预测 vs 停止；target-at-exec | endpoint error；stale/future reject；plan reject；stop latency；消融 | tracking、实时性、数据分析 |
| 第 5 月 | 安全合同与 plan-only near-failure | 保持 plan-only/typed gate/四阶段；故障注入；可选短时 servo sandbox；不改物理成功定义 | phase-wise scorecard；late/cancel/rollback；reject/stop/recovery；独立 contact/lift/retention | MoveIt/ros2_control、安全系统 |
| 第 6 月 | 作品集封装与求职 | benchmark CSV/MCAP、配置/hash、失败分类、设计文档、可重复脚本、短 demo；可选 ACT/DP3/SmolVLA 离线对照 | 一键回放；证据包；结果/局限页；岗位定制简历；无硬件则明确 scope | 完整交付、证据意识、沟通 |

### 半年结束时理想的作品集主张

可以写：

> 在 SO-101 仿真/回放与指定软件协议下，建立了带时间戳、坐标系、epoch、模型 digest 的感知/策略适配器；对目标新鲜度、动态延迟、MoveIt plan-only、typed trajectory gate 和 fail-closed 停止进行了可重复故障注入与分阶段评测。

只有真实配对试验达到定义，才增加：

> 在明确对象、试验分母和保持窗口下，验证了指定硬件配置的物理抓取结果。

不能写：

> 已证明通用 VLA、真实机器人安全或 sim-to-real 成功。

---

## 13. 尚未验证项与后续核对清单

1. 本次没有下载、运行或复现任何外部代码、权重、数据集或模型；所有性能数字均来自论文/官方项目。
2. 未验证任何外部模型在当前 SO-101 URDF、夹爪、相机、Windows/Ubuntu 或目标端硬件上的兼容性、许可证和资源需求。
3. 未运行真实相机、真实机械臂、Jetson、ROS/Gazebo 或物理急停；没有新增真实 calibration、sim-to-real 或 hardware stop 证据。
4. Jetson-PI 的正式同行评审 venue 尚未核验；VLAff 的 arXiv comments 注明 accepted IROS 2026，但正式 proceedings 与代码/数据状态未独立核验，因此仍按 PP 处理。
5. AnyTouch 2 与 RoboTwin 2.0 已分别按 ICLR 2026 和 ICML 2026 更新为 PR；它们的作者指标、跨硬件范围和实际部署仍未在本项目复现。
6. GraspGen/GraspGen-X 的数据量与开放项随仓库版本变化；后续实际采用前应锁定 commit、模型、数据版本和许可证。
7. 公司系统（Gemini Robotics、On-Device、Figure Helix、GR00T 产品栈）的公开指标、频率和适配样本数未获独立复现；不能作选型验收值。
8. 岗位页面是 2026-08-27 的技能信号快照，可能过期；投递前应重新检查岗位状态、地点、学历/年限和社招资格。
9. 若未来进入真实硬件阶段，需另做危险分析、工作空间/速度限制、控制器与硬件停止 acknowledgement、实际 E-stop、标定和现场测试；本文不是安全认证或合规意见。

---

## 14. 最终建议

对 EdgeGrasp，近期最优路线不是“追最大模型”，而是把已有安全协议变成一个可替换模型的**可信实验台**：

1. 先完成当前仿真物理负结果的可重复诊断，保持 <code>physics_grasp_verified=false</code>；
2. 建立静态 RGB-D/pose 的经典基线和完整时间链；
3. 接入一个轻量抓取/pose 模型作为 typed candidate generator；
4. 做动态目标、延迟、遮挡、掉帧和时钟异常的消融；
5. 在目标端做精度—延迟—资源—长稳 scorecard；
6. 最后才比较 ACT/DP3/SmolVLA 或更大 VLA，并始终保留 plan-only、typed gate、freshness、独立 outcome observer 和 fail-closed。

这条路线既吸收了 VLA、Diffusion/Flow、World Model 和多模态前沿，又把个人既有的嵌入式、中间件、接口和可靠性能力转化成大公司机器人算法/端侧 AI 岗真正可验证的作品证据。
