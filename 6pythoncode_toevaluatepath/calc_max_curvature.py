import openpyxl
import numpy as np
import os 

# エクセルファイルがあるフォルダのパス
folder_path = "4research_APF_fig\excel_data\dynamic_3method\origional_dynamic"

# フォルダ内のすべてのファイルを列挙
for filename in os.listdir(folder_path):
    if filename.endswith(".xlsx"):
        # エクセルファイルのパス
        excel_file_path = os.path.join(folder_path, filename)

        try:
            # エクセルファイルを読み込む
            workbook = openpyxl.load_workbook(excel_file_path)
            sheet = workbook.active
            # 列Aと列Bのデータを取得し、数値型に変換
            x_values = [float(cell.value) if cell.value is not None else 0.0 for cell in sheet['A'][1:]]  # 1行目をスキップ
            y_values = [float(cell.value) if cell.value is not None else 0.0 for cell in sheet['B'][1:]]  # 1行目をスキップ

            # 曲率を計算する関数
            def calculate_curvature(x, y):
                A = np.diff(x)
                B = np.diff(y)
                E = A[:-1] * B[1:] - A[1:] * B[:-1]
                F = A[:-1]**2 + B[:-1]**2
                G = A[1:]**2 + B[1:]**2
                H = A[:-1] + B[1:] + A[1:] * B[:-1] - A[:-1] * B[1:]

                # ゼロで割り算が発生しないようにする
                with np.errstate(divide='ignore', invalid='ignore'):
                    curvature = np.zeros_like(E)
                    non_zero_indices = (F * G * H) != 0
                    curvature[non_zero_indices] = 4 * E[non_zero_indices] / (F[non_zero_indices] * G[non_zero_indices] * H[non_zero_indices])

                return curvature

            # 曲率を計算
            curvature_values = calculate_curvature(np.array(x_values), np.array(y_values))

            # 最大曲率のインデックスを取得
            max_curvature_index = np.argmax(np.abs(curvature_values))

            # 経路長を計算
            path_length = np.sum(np.sqrt(np.diff(x_values)**2 + np.diff(y_values)**2))

            # 平均曲率を計算
            # 曲率がゼロでない場合のみ平均を計算
            if not np.all(curvature_values == 0):
                average_curvature = np.nanmean(curvature_values)
            else:
                average_curvature = 'All curvatures are zero'

            # 結果をエクセルのP列に書き込む
            sheet['P1'] = 'Calculated Curvature'
            sheet['P2'] = 'Max Curvature Index'
            sheet['P3'] = 'Path Length'
            sheet['P4'] = 'Average Curvature'
            for i, (curvature, x) in enumerate(zip(curvature_values, x_values), start=5):
                sheet[f'P{i}'] = curvature

            # 最大曲率のインデックスも書き込む
            sheet['Q2'] = max_curvature_index
            sheet['Q3'] = path_length

            # 平均曲率も書き込む
            sheet['R2'] = average_curvature

            # エクセルファイルを保存
            workbook.save(excel_file_path)

            # エクセルファイルを閉じる
            workbook.close()

        except Exception as e:
            print(f"Error processing {excel_file_path}: {e}")

