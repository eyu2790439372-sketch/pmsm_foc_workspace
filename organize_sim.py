import os
import shutil

# 定义重命名映射规则 (源文件 -> 目标路径)
mapping = {
    # 物理域
    "DAY6.m": "sim/plant/pmsm_dq_model.m",
    "DAY8.m": "sim/plant/inverter_3phase.m",
    # 控制域
    "DAY1.m": "sim/control/clarke_transform.m",
    "DAY2.m": "sim/control/park_transform.m",
    "DAY3.m": "sim/control/svpwm_generator.m",
    "DAY4.m": "sim/control/cascade_pid.m",
    "DAY10.m": "sim/control/cascade_pid_decouple.m",
    "DAY11.m": "sim/control/speed_loop_tuning.m",
    "DAY21.m": "sim/control/fsm_controller.m",
    # 观测域
    "DAY5.m": "sim/observer/smo_observer.m",
    "DAY12.m": "sim/observer/smo_c_impl.m",
    "DAY13.m": "sim/observer/sensorless_switch.m",
    "DAY14.m": "sim/observer/startup_strategy.m",
    # 定点化域
    "DAY16.m": "sim/fixed_point/q_format_config.m",
    "DAY17.m": "sim/fixed_point/fixed_transforms.m",
    "DAY18.m": "sim/fixed_point/fixed_svpwm.m",
    "DAY19.m": "sim/fixed_point/fixed_pi.m",
    "DAY20.m": "sim/fixed_point/fixed_smo_pll.m",
    # 验证域
    "DAY7.m": "sim/verification/run_floating_foc.m",
    "DAY9.m": "sim/verification/run_sensorless_foc.m",
    "DAY15.m": "sim/verification/benchmark_floating.m",
    "DAY22.m": "sim/verification/test_fixed_residual.m",
    "DAY23.m": "sim/verification/test_stress_cases.m",
    "DAY30.m": "sim/verification/generate_benchmarks.m",
}


def organize():
    sim_dir = "sim"
    if not os.path.exists(sim_dir):
        print("未找到 sim 文件夹，请确认是否在项目根目录下运行！")
        return

    # 创建目标子文件夹
    subfolders = ["plant", "control", "observer", "fixed_point", "verification"]
    for folder in subfolders:
        os.makedirs(os.path.join(sim_dir, folder), exist_ok=True)

    copied_count = 0
    # 开始复制并重命名
    for src_name, target_path in mapping.items():
        src_path = os.path.join(sim_dir, src_name)
        if os.path.exists(src_path):
            shutil.copy2(src_path, target_path)  # copy2 会保留文件修改时间等属性
            print(f"已复制: {src_name} -> {target_path}")
            copied_count += 1
        else:
            print(f"未找到原文件: {src_name} (已跳过)")

    print(
        f"\n处理完成！共复制并重新归类了 {copied_count} 个文件。"
    )
    print("原来的 DAY1~DAY30 文件已完整保留在 sim 根目录下！")


if __name__ == "__main__":
    organize()