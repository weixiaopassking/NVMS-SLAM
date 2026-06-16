import matplotlib.pyplot as plt
import numpy as np

# 读取文本文件
with open('../config/log_time.txt', 'r') as f:
    data = f.readlines()

# 处理标题行
data0 = [line.strip() for line in data if line.strip() and line.startswith('build frame')]
data0 = [[v.strip() for v in line.split(',')] for line in data0]
d = len(data0[0])-1
print("record time size: ")
print(d)

# 处理数据行
data = [line.strip() for line in data if line.strip() and not line.startswith('build frame')]
data = [[float(v) if v.strip() else None for v in line.split(',')] for line in data]

# 获取每一列的数据
alldata = []
for i in range(d):
    column_data = [row[i] for row in data if row[i] is not None]
    alldata.append(column_data)

# 计算每列的平均值
averages = [np.mean(col) for col in alldata]

# 打印所有列的平均值
print("\n" + "="*60)
print("{:<15} {:<10} {:<15}".format("列名", "样本数", "平均值(ms)"))
print("-"*45)
for i, avg in enumerate(averages):
    col_name = data0[0][i]
    sample_count = len(alldata[i])
    print("{:<15} {:<10} {:.4f}".format(col_name, sample_count, avg))
print("="*60)

# 颜色数组
color_vec = ['blue','red','green','orange','purple','brown','pink','blueviolet','gold','yellow','gray','olivedrab','darkgoldenrod','forestgreen','skyblue']

# 指定需要显示的数据
plot_list=[0,1,2,3,4,5,6,7,8,9,10,11]

# 画折线图
plt.figure(figsize=(12, 6))
for i in plot_list:
    plt.plot(alldata[i], color_vec[i], label=data0[0][i])

# 显示网格
plt.grid(linestyle=":", color="lightgray")
plt.legend(loc='best', ncol=3)
plt.xlabel('Sample')
plt.ylabel('Time(ms)')
plt.title('Time Comparison')

# 画箱线图并标注平均值
plt.figure(figsize=(14, 8))
data_for_box = [alldata[i] for i in plot_list]
label_box = [data0[0][i] for i in plot_list]

# 创建箱线图
box = plt.boxplot(data_for_box,
            medianprops = {'color': 'red', 'linewidth': 2}, 
            meanline = True,    
            showmeans = True,
            meanprops = {'color': 'blue', 'ls': '--', 'linewidth': 2},
            flierprops = {"marker": "o", "markersize": 5, "markerfacecolor": "red", "alpha": 0.5},
            patch_artist=True,
            boxprops={'facecolor': 'lightgreen', 'edgecolor': 'black', 'alpha': 0.7},
            labels = label_box)

plt.grid(True, linestyle=':', alpha=0.7)
plt.xticks(rotation=45, ha='right')  # 旋转X轴标签
plt.ylabel('Time(ms)')
plt.title('Time Comparison with Mean Values')

# 在箱线图上标注平均值
for i, (col, avg) in enumerate(zip(data_for_box, averages)):
    # 获取该箱线图的位置
    pos = i + 1
    
    # 在平均值线上方添加文本
    plt.text(pos, avg + 0.2, f'{avg:.2f} ms', 
             ha='center', va='bottom', 
             fontsize=9, color='blue',
             bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', boxstyle='round,pad=0.2'))

    # 在箱体下方添加列名和样本数
    sample_count = len(col)
    plt.text(pos, min(col) - 0.5, f'n={sample_count}', 
             ha='center', va='top', 
             fontsize=8, color='black')

# 调整布局防止标签被裁剪
plt.tight_layout()
plt.show()