import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rcParams, cm

# ====== 严格学术风格配置 ======
rcParams['font.family'] = 'Arial'  # 期刊常用英文字体，学术规范
rcParams['font.size'] = 12
rcParams['axes.linewidth'] = 1.0
rcParams['figure.dpi'] = 300
rcParams['xtick.direction'] = 'in'
rcParams['ytick.direction'] = 'in'

# ====== 读取本地CSV文件（你的指定方式） ======
df = pd.read_csv('experiment_results.csv', sep=',')

# ====== 数据整理（核心：互换PopulationSize和MigrationRate） ======
# 提取唯一参数并排序
unique_mig = np.sort(df['MigrationRate'].unique())  # 原Y轴，改为X轴
unique_pop = np.sort(df['PopulationSize'].unique())  # 原X轴，改为Y轴

# 构建网格：X=MigrationRate，Y=PopulationSize（实现坐标轴互换）
X, Y = np.meshgrid(unique_mig, unique_pop)  # 顺序互换：先mig，后pop
Z = np.zeros_like(X, dtype=np.float64)

# 填充Z网格（适配互换后的X/Y）
for i, mig_val in enumerate(unique_mig):  # 先遍历迁移率（新X轴）
    for j, pop_val in enumerate(unique_pop):  # 再遍历种群规模（新Y轴）
        target_fitness = df[(df['MigrationRate'] == mig_val) & 
                            (df['PopulationSize'] == pop_val)]['Fitness']
        if not target_fitness.empty:
            Z[j, i] = target_fitness.values[0]
        else:
            Z[j, i] = np.nan

# ====== 创建3D曲面图（经典学术配色+无发白问题） ======
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection='3d')

# 核心优化：使用经典学术配色，移除冗余网格线
surf = ax.plot_surface(
    X, Y, Z,
    # 经典学术配色可选（3种主流方案，按需切换，均比viridis更贴合学术期刊风格）
    # cmap=cm.coolwarm,  # 方案1：冷暖色渐变（红→蓝），对比强烈，最常用（推荐）
    cmap=cm.jet,      # 方案2：彩虹渐变（经典学术配色，兼容性强）
    # cmap=cm.Blues,    # 方案3：单色渐变（蓝色系，简洁高级，适合低对比度场景）
    edgecolor=None,    # 关闭网格线，避免发白
    linewidth=0,
    alpha=0.85,        # 适度透明度，兼顾清晰与美观
    antialiased=True
)

# ====== 坐标轴设置（匹配互换后的参数，学术规范） ======
# 轴标签：X=迁移率，Y=种群规模（已互换）
ax.set_xlabel('Migration Rate', fontsize=13, labelpad=10)  # 原Y轴→新X轴
ax.set_ylabel('Population Size', fontsize=13, labelpad=10)  # 原X轴→新Y轴
ax.set_zlabel('Fitness', fontsize=13, labelpad=10)

# 刻度设置（全轴刻度向内，规范整洁）
ax.tick_params(axis='x', labelsize=10, width=0.8, length=4, pad=4, direction='in')
ax.tick_params(axis='y', labelsize=10, width=0.8, length=4, pad=4, direction='in')
ax.tick_params(axis='z', labelsize=10, width=0.8, length=4, pad=4, direction='in')

# 透明背景+浅灰边框（极简学术风格）
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.xaxis.pane.set_edgecolor('lightgray')
ax.yaxis.pane.set_edgecolor('lightgray')
ax.zaxis.pane.set_edgecolor('lightgray')

# 轻量化网格，不干扰主体视觉
ax.grid(True, linestyle='-', alpha=0.2, linewidth=0.5)

# ====== 颜色条（学术图必备，匹配新配色） ======
cbar = fig.colorbar(surf, ax=ax, shrink=0.8, aspect=25, pad=0.1)
cbar.set_label('Fitness', fontsize=12, labelpad=8)
cbar.ax.tick_params(labelsize=10, width=0.8, length=4)

# ====== 最佳视角展示 ======
ax.view_init(elev=25, azim=135)

# ====== 保存与显示 ======
plt.tight_layout()
plt.savefig('academic_3d_fitness_final.png', dpi=300, bbox_inches='tight', facecolor='white')
plt.savefig('academic_3d_fitness_final.pdf', bbox_inches='tight', facecolor='white')
print("✓ 最终版学术3D曲面图已保存（坐标轴互换+经典学术配色）！")
plt.show()