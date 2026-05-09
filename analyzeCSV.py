import pandas as pd
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
import glob
import sys

# 事前実験で求めた摩擦・粘性パラメータ
DAMPING = 0.00
FRICTION_LOSS = 0.05

# 移動平均フィルタ（ノイズ除去）
def smooth(data, window=11):
    if len(data) < window: return data
    return np.convolve(data, np.ones(window)/window, mode='same')

def select_csv_file():
    csv_files = glob.glob("CSV/*.csv")
    if not csv_files:
        print("エラー： CSVファイルが見つかりません。")
        sys.exit()

    print("利用可能なCSVファイル")
    for i, file in enumerate(csv_files):
        print(f"[{i}] {file}")

    while True:
        try:
            choice = input(f"解析するファイル番号を選択してください (0-{len(csv_files)-1}): ")
            choice_idx = int(choice)

            if 0 <= choice_idx < len(csv_files):
                selected_file = csv_files[choice_idx]
                print(f"\n=================================")
                print(f" {selected_file} を解析します")
                print(f"=================================\n")
                return selected_file
            else:
                print("範囲外です。もう一度入力して下さい。")
        except ValueError:
            print("数字を入力して下さい。")

def calculate_constants():
    csv_file = select_csv_file()

    try:
        # CSV読み込み
        df = pd.read_csv(csv_file)
        df = df.drop_duplicates(subset=['Time[ms]'])
        time_ms = df['Time[ms]'].values
        time_sec = time_ms / 1000.0

        velocity = df['Velocity[rad/s]'].values
        torque = df['Torque[Nm]'].values
        
        mask = time_sec >= 5.0
        time_sec = time_sec[mask]
        velocity = velocity[mask]
        torque = torque[mask]
        if len(time_sec) == 0:
            print("エラー: 5秒以降のデータが存在しません。")
            return
        
        # 加速度算出
        acceleration = np.gradient(velocity, time_sec)

        # スムージング
        v_f = smooth(velocity)
        acc_f = smooth(acceleration)
        tau_f = smooth(torque)

        # 相互相関による遅延推定
        correlation = np.correlate(acc_f - np.mean(acc_f), tau_f - np.mean(tau_f), mode='full')
        lags = np.arange(-len(acc_f) + 1, len(acc_f))
        best_lag = lags[np.argmax(correlation)]

        # ★ 修正ポイント1：時間軸（time_sec）と生トルクも一緒にスライスする
        if best_lag > 0:
            tau_s = tau_f[best_lag:]
            acc_s = acc_f[:-best_lag]
            v_s = v_f[:-best_lag]
            time_s = time_sec[:-best_lag]          # 時間軸を合わせる
            torque_raw_s = torque[best_lag:]       # 比較用に生トルクも合わせる
        elif best_lag < 0:
            tau_s = tau_f[:best_lag]
            acc_s = acc_f[-best_lag:]
            v_s = v_f[-best_lag:]
            time_s = time_sec[-best_lag:]          # 時間軸を合わせる
            torque_raw_s = torque[:best_lag]       # 比較用に生トルクも合わせる
        else:
            tau_s = tau_f
            acc_s = acc_f
            v_s = v_f
            time_s = time_sec
            torque_raw_s = torque
        
        # 既知の摩擦分をトルクから引く
        tau_adj = tau_s - (DAMPING * v_s + FRICTION_LOSS * np.sign(v_s))

        # 最小二乗法で J (armature) と オフセット を推定
        Y = np.vstack([acc_s, np.ones_like(acc_s)]).T
        theta, residuals, rank, s = np.linalg.lstsq(Y, tau_adj, rcond=None)

        armature = theta[0]
        offset = theta[1]

        # 精度評価 ($R^2$)
        prediction = armature * acc_s + offset
        ss_res = np.sum((tau_adj - prediction)**2)
        ss_tot = np.sum((tau_adj - np.mean(tau_adj))**2)
        r_squared = 1 - (ss_res/ss_tot)

        # 結果出力
        print(f"--- 解析完了 ---")
        print(f"対象ファイル: {csv_file}")
        print(f"データ点数: {len(time_sec)}")
        print(f"推定遅延: {best_lag} step (約 {best_lag * np.mean(np.diff(time_sec))*1000:.2f} ms)")
        print(f"----------------")
        print(f"Armature (I_arm): {armature:.8f} kg*m^2")
        print(f"Torque Offset:    {offset:.6f} Nm")
        print(f"決定係数 (R^2):   {r_squared:.4f}")
        print(f"best lag{best_lag:.8f} s")

        # ★ 修正ポイント2：推定したJ、摩擦、オフセットをすべて足し合わせて推測トルクを再構築する
        t_fit = (armature * acc_s) + (DAMPING * v_s) + (FRICTION_LOSS * np.sign(v_s)) + offset

        # グラフ描画
        plt.figure(figsize=(10, 6))
        
        # 生のトルクデータ（薄く表示）
        plt.plot(time_s, torque_raw_s, label='Measured Torque (Raw, Shifted)', color='blue', alpha=0.3)
        # スムージングされたトルクデータ
        plt.plot(time_s, tau_s, label='Measured Torque (Smoothed)', color='green', alpha=0.6)
        # 計算から導き出したトルクモデル
        plt.plot(time_s, t_fit, label='Fitted Model Torque', color='red', linestyle='--')

        plt.xlabel('Time [s]')
        plt.ylabel('Torque [Nm]')
        plt.title(f'System Identification - {csv_file}')
        plt.legend()
        plt.grid(True)
        plt.show()

    except Exception as e:
        print(f"解析中にエラーが発生しました: {e}")

if __name__ == "__main__":
    calculate_constants()