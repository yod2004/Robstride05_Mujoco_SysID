import serial
import serial.tools.list_ports
import math
import sys
import threading
import csv
import datetime
import os

def select_com_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COMポートが見つかりません")
        sys.exit()
    
    print("利用可能なCOMポート")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")

    while True:
        try:
            choice = input(f"接続する番号を選択して下さい (0-{len(ports)-1}): ")
            choice_idx = int(choice)

            if 0 <= choice_idx < len(ports):
                selected_port = ports[choice_idx].device
                print(f"\n{selected_port} に接続します\n")
                return selected_port
            else:
                print("範囲外です．もう一度入力して下さい．")
        except ValueError:
            print("数字を入力して下さい．")

def keyboard_listener(ser):
    while True:
        try:
            user_input = input()
            if user_input == '' or user_input == 's':
                ser.write(b's')
                print("\n>>>[PC]'s'を送信しました<<<\n")
            elif user_input == 'e':
                ser.write(b'e')
                print("\n>>>[PC]'e'を送信しました<<<\n")
        except EOFError:
            break
        except Exception as e:
            print(f"送信エラー: {e}")
            break

def main():
    port_name=select_com_port()

    try:
        ser = serial.Serial(port_name, 460800)
    except serial.SerialException as e:
        print(f"ポートを開けませんでした: {e}")
        sys.exit()

    dt_now = datetime.datetime.now()
    filename = dt_now.strftime("datalog_%Y%m%d_%H%M%S.csv")
    filepath = os.path.join("CSV", filename)
    
    try:
        csv_file = open(filepath, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["Time[ms]", "Angle[rad]", "Velocity[rad/s]", "Torque[Nm]", "Temp[C]"])
        print(f"[{filepath}] にデータを記録します．")
    except Exception as e:
        print(f"CSVファイルの作成に失敗しました: {e}")
        sys.exit()
    
    print("========================================")
    print(" 受信待機中 (Ctrl+Cで終了)")
    print(" 【操作】エンターキーを押すと 's' を送信します")
    print("         'e' を入力してエンターで 'e' を送信します")
    print("========================================\n")

    k_thread = threading.Thread(target=keyboard_listener, args=(ser,), daemon=True)
    k_thread.start()

    buffer = bytearray()
    try:
        while True:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

            while len(buffer) >= 12:
                rx_data = buffer[:12]
                del buffer[:12]
                
                timer_ms = (rx_data[3]<<24|rx_data[2]<<16|rx_data[1]<<8|rx_data[0])
                angle_int =    (rx_data[4]<<8)|rx_data[5]
                velocity_int = (rx_data[6]<<8)|rx_data[7]
                torque_int =   (rx_data[8]<<8)|rx_data[9]
                temp_int =     (rx_data[10]<<8)|rx_data[11]

                timer = timer_ms / 1000
                angle = angle_int * 8 * math.pi / 65535 - 4 * math.pi
                velocity = velocity_int * 100 / 65535 - 50
                torque = torque_int * 11 /65535 - 5.5
                temp = temp_int / 10

                # print(f"Timer: {timer: 7.2f} s, Angle: {angle:7.2f} rad, Vel: {velocity:6.2f} rad/s, Torque: {torque:5.2f} Nm, Temp: {temp:4.1f} °C")
                csv_writer.writerow([timer_ms, angle, velocity, torque, temp])

    except KeyboardInterrupt:
        print("\n通信を終了し，CSVファイルを保存します．")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        if 'csv_file' in locals() and not csv_file.closed:
            csv_file.close()

if __name__ == "__main__":
    main()