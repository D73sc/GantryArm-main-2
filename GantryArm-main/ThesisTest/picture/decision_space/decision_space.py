import pandas as pd
import matplotlib.pyplot as plt
import os
from pathlib import Path

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 1.2

def plot_decision_space(csv_file, output_file):
    df = pd.read_csv(csv_file)
    valid = df[df['has_valid_solution'] == True]
    
    fig, ax = plt.subplots(figsize=(6, 6), dpi=300)
    
    ax.scatter(valid['axis1'], valid['axis2'], 
              c='steelblue', s=15, alpha=0.7, edgecolors='none')
    
    ax.set_xlabel(r'$\theta_1$ (rad)', fontsize=11)
    ax.set_ylabel(r'$\theta_2$ (rad)', fontsize=11)
    
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

# 处理results文件夹
results_dir = Path('results')
methods = ['47', '45', 'auto']

for method in methods:
    method_dir = results_dir / method
    if not method_dir.exists():
        print(f"⚠ {method_dir} 不存在")
        continue
    
    csv_files = sorted(method_dir.glob('point_*.csv'))
    print(f"\n处理方案 {method}: {len(csv_files)} 个文件")
    
    for i, csv_file in enumerate(csv_files):
        output_file = csv_file.with_suffix('.png')
        plot_decision_space(str(csv_file), str(output_file))
        
        if (i + 1) % 50 == 0:
            print(f"  ✓ 已完成 {i + 1}/{len(csv_files)}")
    
    print(f"  ✓ 方案 {method} 完成")

print("\n✓ 所有PNG已生成")