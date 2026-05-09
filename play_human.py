import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

# ※もし外部のXMLファイルを読み込みたい場合は、以下のように書き換えてください：
model = mujoco.MjModel.from_xml_path("RS05.xml")
data = mujoco.MjData(model)
JOINT_NAME = "RS05"
ACTUATOR_NAME = "RS05"

# ジョイントのIDを取得
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT_NAME)
if joint_id == -1:
    print(f"エラー: '{JOINT_NAME}' という名前のジョイントがXMLに見つかりません！")
    exit()

# モーター(アクチュエータ)のIDを取得
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ACTUATOR_NAME)
if actuator_id == -1:
    print(f"エラー: '{ACTUATOR_NAME}' という名前のアクチュエータがXMLに見つかりません！")
    exit()

# 配列の中でモーターが何番目に格納されているかを取得
qpos_idx = model.jnt_qposadr[joint_id]
qvel_idx = model.jnt_dofadr[joint_id]
ctrl_idx = actuator_id

def main():
    print("=== MuJoCo シミュレーション開始 ===")
    print("3Dビューアのウィンドウを閉じると、途中で終了してグラフを描画します。")

    # 記録用リスト
    time_history = []
    angle_history = []
    velocity_history = []
    torque_history = []

    # 入力波形のパラメータ
    f = 4.0
    duration = 30.0

    # --- 制御周期の設定 ---
    control_dt = 0.001  # 制御周期を1ms (0.001秒) に設定
    next_control_time = 0.0

    # 描画のフレームレート調整用
    step_count = 0
    step_start = time.time()

    # 3Dビューアを起動してシミュレーション開始
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        while data.time < duration and viewer.is_running():
            
            # --- 2. 制御の計算とデータの記録 (10msごとに実行) ---
            # 浮動小数点の計算誤差を考慮して微小値(1e-6)を引いて比較します
            if data.time >= next_control_time - 1e-6:
                t = data.time

                # トルクの計算 (マルチサイン波)
                tau = 0.137 * (np.sin(2 * np.pi * f * t) + 
                             0.6 * np.sin(2 * np.pi * 3.4 * f * t) + 
                             0.3 * np.sin(2 * np.pi * 7.4 * f * t))
                
                if t < 6:
                    speed = 2 * int(t)
                elif t < 10:
                    speed = 2 * int(11 - t)
                else:
                    speed = 0

                # アクチュエータに指令値をセット
                data.ctrl[ctrl_idx] = speed
                
                # データの記録
                time_history.append(t)
                angle_history.append(data.qpos[qpos_idx])  # 角度
                velocity_history.append(data.qvel[qvel_idx]) # 角速度
                # torque_history.append(tau)            # 入力トルク

                # 次の制御タイミングを更新
                next_control_time += control_dt

            # --- 3. 物理シミュレーション ---
            # シミュレーション自体はモデルのデフォルトtimestep(通常1msや2ms)で1ステップ進める
            mujoco.mj_step(model, data)
            step_count += 1

            # --- 4. 画面の更新（描画の実時間同期） ---
            # データ記録リストの長さではなく、物理ステップの回数(step_count)を基準にする
            if step_count % 16 == 0:
                viewer.sync()
                
                # 実時間と同じスピードで進むように少し待機 (速すぎ防止)
                time_until_next = model.opt.timestep * 16 - (time.time() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)
                step_start = time.time()

    print("\n=== 計測終了。グラフを描画します ===")

    # =======================================================
    # 5. グラフの描画
    # =======================================================
    df = pd.DataFrame({
        'Time_s': time_history,
        'Angle_rad': angle_history,
        'Velocity_rads': velocity_history
    })
    csv_filename = "simulation_results.csv"
    df.to_csv(csv_filename, index=False)
    print(f"CSVファイルを保存しました: {csv_filename}")


    # plt.figure(figsize=(10, 10))
    plt.figure(figsize=(10, 7))

    # ① 角度のグラフ
    # plt.subplot(3, 1, 1)
    plt.subplot(2, 1, 1)
    plt.plot(time_history, angle_history, label='Angle [rad]', color='blue')
    plt.ylabel('Angle [rad]')
    plt.title('MuJoCo Bare Motor Simulation (Control Cycle: 1ms)')
    plt.grid(True)
    plt.legend()

    # ② 角速度のグラフ
    # plt.subplot(3, 1, 2)
    plt.subplot(2, 1, 2)
    plt.plot(time_history, velocity_history, label='Velocity [rad/s]', color='green')
    plt.ylabel('Velocity [rad/s]')
    plt.grid(True)
    plt.legend()

    # # ③ トルクのグラフ
    # plt.subplot(3, 1, 3)
    # plt.plot(time_history, torque_history, label='Input Torque [Nm]', color='red')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Torque [Nm]')
    # plt.grid(True)
    # plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()