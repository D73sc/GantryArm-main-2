import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.pyplot as plt

# 1) 中文显示不乱码：优先用 Windows/macOS 常见字体；你可按系统调整
matplotlib.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "PingFang SC", "Arial Unicode MS"]
matplotlib.rcParams["axes.unicode_minus"] = False  # 解决负号乱码
# 读取数据
df = pd.read_excel("force_data.xlsx", sheet_name="Sheet2")

# 三次实验（根据你表格列名）
cols = ["Fz(N)_1", "Fz(N)_2", "Fz(N)_3"]

# 如果你希望横轴是采样点序号；若你有时间列，可替换
x = range(len(df))

plt.figure(figsize=(10, 5.2))

for c in cols:
    plt.plot(x, df[c], linewidth=1.8, label=c)

# 参考接触力（论文期望设定为 -40 N）
plt.axhline(-40, color="k", linestyle="--", linewidth=1.2, label="reference: -40 N")
plt.xlim(0, len(df)-1)
plt.xlabel("采样点序号", fontsize=12)
plt.ylabel("法向接触力 $F_z$ (N)", fontsize=12)
#plt.title("实际恒力控制实验：三次重复实验接触力对比")
plt.grid(True, alpha=0.25)
plt.legend(loc="best")

plt.tight_layout()
plt.savefig("actual_force_control_result_three_trials.pdf")
plt.show()