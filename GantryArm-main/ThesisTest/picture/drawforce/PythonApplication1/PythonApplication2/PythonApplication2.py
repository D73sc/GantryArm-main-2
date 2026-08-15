
import pandas as pd
import numpy as np

df = pd.read_excel("force_data.xlsx", sheet_name="Sheet2")

cols = ["Fz(N)_1", "Fz(N)_2", "Fz(N)_3"]
target = -40.0

# 你的稳定区间（采样点索引）
stable_segments = [
    ("稳定段1", 4, 40),
    ("稳定段2", 54, 87),
    ("稳定段3", 100, len(df)-1),
]

def calc_segment(seg_df):
    out = []
    for c in cols:
        y = seg_df[c].astype(float).values
        out.append({
            "试验": c,
            "均值(N)": float(np.mean(y)),
            "标准差(N)": float(np.std(y, ddof=1)) if len(y) > 1 else 0.0,
            "MAE(N)": float(np.mean(np.abs(y - target))),
            "最小值(N)": float(np.min(y)),
            "最大值(N)": float(np.max(y)),
        })
    return pd.DataFrame(out)

for name, a, b in stable_segments:
    seg_df = df.iloc[a:b+1]
    tab = calc_segment(seg_df)
    print("\n" + "="*30)
    print(name, "区间:", a, "~", b, "(长度:", len(seg_df), ")")
    print(tab.to_string(index=False))