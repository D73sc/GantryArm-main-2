import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams, cm

# ====== 学术风格配置（保持统一规范） ======
rcParams['font.family'] = 'Arial'
rcParams['font.size'] = 12
rcParams['axes.linewidth'] = 1.0
rcParams['figure.dpi'] = 300
rcParams['xtick.direction'] = 'in'
rcParams['ytick.direction'] = 'in'

# ====== 读取本地CSV文件 ======
df = pd.read_csv('experiment_results.csv', sep=',')

# ====== 数据整理（仅修改inf填充逻辑，其他完全保留） ======
# 提取唯一参数并排序
unique_pop = np.sort(df['PopulationSize'].unique())  # 横轴：种群规模（横向）
unique_mig = np.sort(df['MigrationRate'].unique())    # 纵轴：迁移率（纵向）
n_mig = len(unique_mig)
n_pop = len(unique_pop)

# 核心修改1：过滤无穷大值（inf），获取有效Fitness数据（原有逻辑保留）
valid_fitness = df['Fitness'][np.isfinite(df['Fitness'])]  # 过滤inf
valid_fitness = valid_fitness.dropna()  # 过滤NaN，仅保留有效数值
fitness_min = valid_fitness.min()  # 基于有效数据计算最小值，避免inf干扰
print(f"有效Fitness最小值：{fitness_min}，仅用于填充NaN值")

# 第一步：先构建原始矩阵（区分NaN和inf，先按原有逻辑填充NaN，标记inf位置）
heatmap_data = np.zeros((n_mig, n_pop), dtype=np.float64)
inf_positions = []  # 记录inf值的坐标(i,j)
for i, mig_val in enumerate(unique_mig):
    for j, pop_val in enumerate(unique_pop):
        target_fitness = df[(df['PopulationSize'] == pop_val) & 
                            (df['MigrationRate'] == mig_val)]['Fitness']
        if target_fitness.empty:
            # NaN值：仍用全局最小值填充（原有逻辑不变）
            heatmap_data[i, j] = fitness_min
        else:
            fitness_val = target_fitness.values[0]
            if not np.isfinite(fitness_val):
                # 先标记inf位置，后续统一处理为相邻均值
                inf_positions.append((i, j))
                # 临时填充最小值，后续会替换
                heatmap_data[i, j] = fitness_min
            else:
                # 正常数值：直接保留（原有逻辑不变）
                heatmap_data[i, j] = fitness_val

# 第二步：仅处理inf值，替换为周围两个有效数据的均值（核心修改）
for (i, j) in inf_positions:
    neighbor_values = []
    # 获取横向相邻数据（左侧j-1、右侧j+1，优先横向）
    if j - 1 >= 0:
        neighbor_values.append(heatmap_data[i, j-1])
    if j + 1 < n_pop:
        neighbor_values.append(heatmap_data[i, j+1])
    # 若横向无数据，获取纵向相邻数据（上方i-1、下方i+1）
    if len(neighbor_values) == 0:
        if i - 1 >= 0:
            neighbor_values.append(heatmap_data[i-1, j])
        if i + 1 < n_mig:
            neighbor_values.append(heatmap_data[i+1, j])
    
    # 填充inf值：有相邻数据取均值，无相邻数据则保留最小值（容错）
    if len(neighbor_values) >= 1:
        inf_fill_val = np.mean(neighbor_values)
        heatmap_data[i, j] = inf_fill_val
        print(f"位置(i={i}, j={j})的inf值，已替换为相邻数据均值：{inf_fill_val:.2f}")
    # 无相邻数据时，仍保留最小值（避免报错，兼容边界情况）

# ====== 创建热力图（完全保留原有逻辑，未做任何改动） ======
fig, ax = plt.subplots(figsize=(12, 8))

# 绘制核心简洁热力图（仅颜色展示，无密集数值）
im = ax.imshow(
    heatmap_data,
    cmap=cm.jet,  # 经典学术冷暖色渐变，快速区分Fitness高低
    aspect='auto',     # 自动适配长宽比，避免变形
    alpha=0.9,         # 适度透明度，色彩更柔和
    origin='upper'     # 符合阅读习惯
)

# ====== 坐标轴设置（完全保留原有逻辑，未做任何改动） ======
# 横轴：Population Size（10的倍数刻度，水平显示）
pop_10x = [p for p in unique_pop if p % 10 == 0]
pop_10x_indices = [np.where(unique_pop == p)[0][0] for p in pop_10x]
ax.set_xticks(pop_10x_indices)
ax.set_xticklabels([f'{int(p)}' for p in pop_10x], 
                   fontsize=10, rotation=0, ha='center')

# 纵轴：Migration Rate（核心修改：不依赖原始数据，强制0.1-1.0固定间隔标签）
# 1. 生成固定标签：0.1, 0.2, ..., 1.0
fixed_mig_labels = [f'{x:.1f}' for x in np.arange(0.1, 1.1, 0.1)]
# 2. 计算原始迁移率数据的索引映射：将固定标签对应到原始数据的近似位置
mig_indices = []
for target_mig in np.arange(0.1, 1.1, 0.1):
    # 找到原始迁移率中最接近当前固定目标值的索引
    closest_idx = np.argmin(np.abs(unique_mig - target_mig))
    # 去重（避免原始数据有重复值导致索引重复）
    if closest_idx not in mig_indices:
        mig_indices.append(closest_idx)
# 3. 确保标签数量和索引数量匹配（若原始数据不足，补充至对应长度）
if len(mig_indices) < len(fixed_mig_labels):
    # 填充末尾索引（避免标签缺失）
    mig_indices.extend([len(unique_mig)-1] * (len(fixed_mig_labels) - len(mig_indices)))
# 截断多余索引（确保一一对应）
mig_indices = mig_indices[:len(fixed_mig_labels)]

# 4. 设置纵轴刻度和固定标签（彻底脱离原始数据标签的影响）
ax.set_yticks(mig_indices)
ax.set_yticklabels(fixed_mig_labels, fontsize=10)

# 轴标签与标题
ax.set_xlabel('Population Size', fontsize=13, labelpad=10)
ax.set_ylabel('Migration Rate', fontsize=13, labelpad=10)
# ax.set_title('Fitness Heatmap (Parameter Selection)', fontsize=14, pad=15)

# 刻度样式（学术规范，向内刻度）
ax.tick_params(axis='x', labelsize=10, width=0.8, length=4, pad=4, direction='in')
ax.tick_params(axis='y', labelsize=10, width=0.8, length=4, pad=4, direction='in')

# 辅助网格线（精准定位参数，不干扰颜色视觉）
ax.grid(True, which='minor', linestyle='-', alpha=0.4, linewidth=0.6, color='gray')
ax.set_xticks(np.arange(len(unique_pop)+1)-0.5, minor=True)
ax.set_yticks(np.arange(len(unique_mig)+1)-0.5, minor=True)
ax.set_xlim(-0.5, len(unique_pop)-0.5)
ax.set_ylim(-0.5, len(unique_mig)-0.5)

# ====== 颜色条（对应Fitness值，辅助参数筛选） ======
cbar = fig.colorbar(im, ax=ax, shrink=0.8, aspect=25, pad=0.05)
cbar.set_label('Fitness', fontsize=12, labelpad=8)
cbar.ax.tick_params(labelsize=10, width=0.8, length=4, direction='in')

# ====== 保存与显示 ======
plt.tight_layout()
plt.savefig('fitness_final_fixed_2decimals_heatmap.png', dpi=300, bbox_inches='tight', facecolor='white')
plt.savefig('fitness_final_fixed_2decimals_heatmap.pdf', bbox_inches='tight', facecolor='white')
print("✓ 修复版热力图已保存（仅替换inf为相邻均值，其他逻辑不变）！")
plt.show()