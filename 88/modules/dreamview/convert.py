import pandas as pd

def read_and_convert_txt_to_excel(input_file, output_file):
    with open(input_file, "r", encoding="utf-8") as file:
        lines = file.readlines()

    data = []
    current_id = None
    current_coords = ""

    for line in lines:
        line = line.strip()
        if not line:
            continue
        
        if line[0].isdigit() or line.isalnum():
            if current_id is not None:
                data.append([current_id, current_coords.strip()])
            current_id = line
            current_coords = ""
        else:
            current_coords += line + "\n"

    if current_id is not None:
        data.append([current_id, current_coords.strip()])

    df = pd.DataFrame(data, columns=["编号", "坐标 (X, Y, Z)"])
    
    with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
        df.to_excel(writer, index=False, sheet_name='Sheet1')
        
        workbook = writer.book
        worksheet = writer.sheets['Sheet1']

        worksheet.set_column('B:B', 20, None)
        cell_format = workbook.add_format({'text_wrap': True})
        worksheet.set_column('B:B', None, cell_format)

        for col_num, column in enumerate(df.columns):
            max_len = 0
            for row_num in range(len(df)):
                cell_value = str(df.iloc[row_num, col_num])
                max_len = max(max_len, len(cell_value))
            worksheet.set_column(col_num, col_num, max_len + 2)

        for row_num in range(len(df) + 1):
            max_len = 0
            for col_num in range(len(df.columns)):
                cell_value = str(df.iloc[row_num - 1, col_num]) if row_num > 0 else df.columns[col_num]
                max_len = max(max_len, len(cell_value))
            
            worksheet.set_row(row_num, max_len * 1.2)

    print(f"convert successfully, file saved to {output_file}")

input_txt_file = "position.txt"
output_excel_file = "output.xlsx"
read_and_convert_txt_to_excel(input_txt_file, output_excel_file)
