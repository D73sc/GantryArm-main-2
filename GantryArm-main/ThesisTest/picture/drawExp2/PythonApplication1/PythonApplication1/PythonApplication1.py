import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams

# Font settings for IEEE publications
rcParams['font.family'] = 'serif'
rcParams['font.serif'] = ['Times New Roman', 'DejaVu Serif']
rcParams['font.size'] = 10
rcParams['axes.unicode_minus'] = False
rcParams['figure.dpi'] = 100
rcParams['axes.linewidth'] = 1.2
rcParams['grid.alpha'] = 0.2
rcParams['legend.framealpha'] = 0.9

def main():
    # ========== 1. Load Data ==========
    print("Loading data...")
    data = pd.read_csv('method_comparison.csv')

    # Extract all data
    success_47 = data['Method_47_Success_Rate']
    success_45 = data['Method_45_Success_Rate']
    success_auto = data['Method_Auto_Success_Rate']
    time_47 = data['Method_47_Time_ms']
    time_45 = data['Method_45_Time_ms']
    time_auto = data['Method_Auto_Time_ms']

    # Calculate average values
    avg_success_47 = success_47.mean()
    avg_success_45 = success_45.mean()
    avg_success_auto = success_auto.mean()
    avg_time_47 = time_47.mean()
    avg_time_45 = time_45.mean()
    avg_time_auto = time_auto.mean()

    # Plot parameters
    methods = ['Method 1', 'Method 2', 'Dynamic Selection']
    success_data = [success_47, success_45, success_auto]
    time_data = [time_47, time_45, time_auto]
    colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
    x_positions = [1, 2, 3]
    violin_width = 0.6
    label_offset = 0.15

    # ========== Main Figure: Average values labeled next to median line ==========
    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # ---------------- Subplot 1: Success Rate Comparison ----------------
    # 1. Violin plot with clear median line
    violin1 = ax1.violinplot(
        success_data,
        positions=x_positions,
        showmedians=True,
        showextrema=True,
        widths=violin_width,
    )
    # Violin styling
    for i, pc in enumerate(violin1['bodies']):
        pc.set_facecolor(colors[i])
        pc.set_alpha(0.6 if i != 2 else 0.9)
        pc.set_edgecolor('black')
        pc.set_linewidth(1.2)
    # Median line styling
    violin1['cmedians'].set_color('black')
    violin1['cmedians'].set_linewidth(2)
    violin1['cbars'].set_linewidth(1.2)
    violin1['cbars'].set_color('black')

    # 2. Scatter plot for all data points
    for i, (x, data) in enumerate(zip(x_positions, success_data)):
        ax1.scatter(
            [x] * len(data),
            data,
            color=colors[i],
            alpha=0.1 if i != 2 else 0.2,
            s=8,
            edgecolor='none'
        )

    # 3. Average value labels next to median line
    avg_success_list = [avg_success_47, avg_success_45, avg_success_auto]
    for i, (x, avg_val) in enumerate(zip(x_positions, avg_success_list)):
        ax1.text(
            x + label_offset,
            avg_val,
            f'{avg_val:.2f}%',
            ha='left',
            va='center',
            fontsize=9,
            fontweight='bold',
            color=colors[i],
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8)
        )

    # Subplot 1 styling
    ax1.set_xticks(x_positions)
    ax1.set_xticklabels(methods, fontsize=11)
    ax1.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
    ax1.set_ylim([0, 105])
    ax1.grid(True, alpha=0.2, axis='y')

    # ---------------- Subplot 2: Computation Time Comparison ----------------
    # 1. Violin plot
    violin2 = ax2.violinplot(
        time_data,
        positions=x_positions,
        showmedians=True,
        showextrema=True,
        widths=violin_width,
    )
    # Violin styling
    for i, pc in enumerate(violin2['bodies']):
        pc.set_facecolor(colors[i])
        pc.set_alpha(0.6 if i != 2 else 0.9)
        pc.set_edgecolor('black')
        pc.set_linewidth(1.2)
    # Median line styling
    violin2['cmedians'].set_color('black')
    violin2['cmedians'].set_linewidth(2)
    violin2['cbars'].set_linewidth(1.2)
    violin2['cbars'].set_color('black')

    # 2. Scatter plot for all data points
    for i, (x, data) in enumerate(zip(x_positions, time_data)):
        ax2.scatter(
            [x] * len(data),
            data,
            color=colors[i],
            alpha=0.1 if i != 2 else 0.2,
            s=8,
            edgecolor='none'
        )

    # 3. Average value labels next to median line
    avg_time_list = [avg_time_47, avg_time_45, avg_time_auto]
    for i, (x, avg_val) in enumerate(zip(x_positions, avg_time_list)):
        ax2.text(
            x + label_offset,
            avg_val,
            f'{avg_val:.2f}ms',
            ha='left',
            va='center',
            fontsize=9,
            fontweight='bold',
            color=colors[i],
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8)
        )

    # Subplot 2 styling
    ax2.set_xticks(x_positions)
    ax2.set_xticklabels(methods, fontsize=11)
    ax2.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.2, axis='y')
    
    # Save figures
    plt.tight_layout()
    plt.savefig('comparison_violin_avg_next_to_line.png', dpi=300, bbox_inches='tight')
    plt.savefig('comparison_violin_avg_next_to_line.pdf', dpi=300, bbox_inches='tight')
    print("✓ Figures saved: comparison_violin_avg_next_to_line.png and .pdf")

    # ========== Statistical Analysis + LaTeX Code ==========
    print("\n" + "="*80)
    print("Performance Analysis of Dynamic Selection Strategy".center(80))
    print("="*80)
    print(f"{'Metric':<25} | {'Method 1':>10} | {'Method 2':>10} | {'Adaptive selection strategy':>10} | {'Improvement':>12}")
    print("-"*80)

    # Calculate improvement metrics
    max_baseline = max(avg_success_47, avg_success_45)
    success_improvement = (avg_success_auto - max_baseline) / max_baseline * 100
    min_baseline_time = min(time_47.sum(), time_45.sum())
    time_saving = (1 - time_auto.sum() / min_baseline_time) * 100
    min_baseline_std = min(success_47.std(), success_45.std())
    stability_improvement = (1 - success_auto.std() / min_baseline_std) * 100

    # Print statistical comparison
    print(f"{'Average Success Rate (%)':<25} | {avg_success_47:>10.2f} | {avg_success_45:>10.2f} | {avg_success_auto:>10.2f} | {success_improvement:>11.1f}%")
    print(f"{'Total Time (s)':<25} | {time_47.sum()/1000:>10.2f} | {time_45.sum()/1000:>10.2f} | {time_auto.sum()/1000:>10.2f} | {time_saving:>11.1f}%")
    print(f"{'Std Dev (%)':<25} | {success_47.std():>10.2f} | {success_45.std():>10.2f} | {success_auto.std():>10.2f} | {stability_improvement:>11.1f}%")
    print(f"{'Avg Time per Point (ms)':<25} | {avg_time_47:>10.2f} | {avg_time_45:>10.2f} | {avg_time_auto:>10.2f} |")
    print("-"*80)

    # LaTeX table code
    print("\n========== LaTeX Table Code ==========")
    print("\\begin{table}[htbp]")
    print("\\centering")
    print("\\caption{Performance Comparison of Inverse Kinematics Methods}")
    print("\\label{tab:ik_comparison}")
    print("\\begin{tabular}{lcccc}")
    print("\\hline")
    print("Metric & Method 1 & Method 2 & Dynamic & Improvement \\\\")
    print("\\hline")
    print(f"Avg. Success Rate (\\%) & {avg_success_47:.2f} & {avg_success_45:.2f} & {avg_success_auto:.2f} & {success_improvement:+.1f}\\% \\\\")
    print(f"Total Time (s) & {time_47.sum()/1000:.2f} & {time_45.sum()/1000:.2f} & {time_auto.sum()/1000:.2f} & {time_saving:+.1f}\\% \\\\")
    print(f"Std. Deviation (\\%) & {success_47.std():.2f} & {success_45.std():.2f} & {success_auto.std():.2f} & {stability_improvement:+.1f}\\% \\\\")
    print("\\hline")
    print("\\end{tabular}")
    print("\\end{table}")

    print("\n✓ Average values are labeled next to the median line for clear visualization!")

if __name__ == '__main__':
    main()