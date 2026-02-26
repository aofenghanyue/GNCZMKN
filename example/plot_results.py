#!/usr/bin/env python3
"""
绘制 simple_gravity_turn 仿真结果

用法:
    python3 plot_results.py [csv_file]

默认读取: gravity_turn_sim.csv (由 simple_gravity_turn 生成)
"""

import sys
import csv
import os

def read_csv(filename):
    """读取 CSV 文件并返回列名和数据字典"""
    data = {}
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)
        for h in headers:
            data[h.strip()] = []
        for row in reader:
            for h, val in zip(headers, row):
                try:
                    data[h.strip()].append(float(val))
                except ValueError:
                    data[h.strip()].append(0.0)
    return data


def plot_with_matplotlib(data):
    """使用 matplotlib 绘图"""
    import matplotlib.pyplot as plt

    time = data.get('time', [])
    if not time:
        print("Error: no time column found")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('GNC Gravity Turn Simulation Results', fontsize=14)

    # --- 位置 ---
    ax = axes[0][0]
    for key, label in [('dynamics.pos.x', 'X (horizontal)'),
                        ('dynamics.pos.z', 'Z (vertical)')]:
        if key in data:
            ax.plot(time, data[key], label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 速度 ---
    ax = axes[0][1]
    for key, label in [('dynamics.vel.x', 'Vx (horizontal)'),
                        ('dynamics.vel.z', 'Vz (vertical)')]:
        if key in data:
            ax.plot(time, data[key], label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity vs Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 轨迹 (X vs Z) ---
    ax = axes[1][0]
    px = data.get('dynamics.pos.x', [])
    pz = data.get('dynamics.pos.z', [])
    if px and pz:
        ax.plot(px, pz)
        ax.set_xlabel('X - Horizontal (m)')
        ax.set_ylabel('Z - Vertical (m)')
        ax.set_title('Trajectory (X-Z plane)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')

    # --- 制导指令 ---
    ax = axes[1][1]
    for key, label in [('guidance.acc_cmd.x', 'Cmd X'),
                        ('guidance.acc_cmd.z', 'Cmd Z')]:
        if key in data:
            ax.plot(time, data[key], label=label)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration Command (m/s²)')
    ax.set_title('Guidance Commands')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_file = 'gravity_turn_results.png'
    plt.savefig(output_file, dpi=150)
    print(f"Plot saved to: {output_file}")
    plt.show()


def print_text_summary(data):
    """纯文本摘要 (当 matplotlib 不可用时)"""
    time = data.get('time', [])
    if not time:
        print("No data to display")
        return

    print(f"\n{'='*60}")
    print(f"  Gravity Turn Simulation Summary")
    print(f"{'='*60}")
    print(f"  Total time points: {len(time)}")
    print(f"  Time range: {time[0]:.2f} - {time[-1]:.2f} s")

    for key in ['dynamics.pos.x', 'dynamics.pos.z',
                'dynamics.vel.x', 'dynamics.vel.z']:
        if key in data and data[key]:
            vals = data[key]
            print(f"  {key:25s}: final = {vals[-1]:12.3f}  "
                  f"min = {min(vals):12.3f}  max = {max(vals):12.3f}")

    print(f"{'='*60}\n")


def main():
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'gravity_turn_sim.csv'

    if not os.path.exists(csv_file):
        print(f"Error: file '{csv_file}' not found.")
        print("Please run the simulation first: ./bin/simple_gravity_turn")
        sys.exit(1)

    print(f"Reading: {csv_file}")
    data = read_csv(csv_file)

    # 总是先打印文本摘要
    print_text_summary(data)

    # 尝试 matplotlib 绘图
    try:
        import matplotlib
        matplotlib.use('Agg')  # 非交互模式
        plot_with_matplotlib(data)
    except ImportError:
        print("matplotlib not installed. Showing text summary only.")
        print("Install with: pip install matplotlib")


if __name__ == '__main__':
    main()
