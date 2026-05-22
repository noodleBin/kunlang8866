import struct
import pandas as pd
import os
import sys

HEADER_SIZE = 512 * 1024
TASK_COMPUTE_DERIVATIVES = 0
TASK_LOOP = 1
BEGIN, END = 1, 2

def get_files_with_prefix(prefix, directory = "./"):
    file_list = []
    
    for filename in os.listdir(directory):
        if filename.startswith(prefix) and os.path.isfile(os.path.join(directory, filename)):
            file_list.append(filename)
    
    return file_list

def decode_data(trace_file_prefix):
    trace_file_path_list = get_files_with_prefix(trace_file_prefix)

    with open(trace_file_path_list[0], 'rb') as file:
        struct_time = 'qq{}s'.format(HEADER_SIZE - struct.calcsize("qq"))
        base_monotonic, base_system, header_data = struct.unpack(struct_time, file.read(struct.calcsize(struct_time)))
        header_data = header_data.replace(b'\x00', b'').decode('utf-8')
    df_columns = ["Timestamp", "TID", "Time_Type", "Task_Type", "Task_ID"]
    df_data = []

    for trace_file_path in trace_file_path_list:
        with open(trace_file_path, 'rb') as file:
            file.seek(HEADER_SIZE)
            struct_format = 'qI4xiiQ32x' # derived based on struct TraceData memory allocation
            while True:
                struct_data = file.read(struct.calcsize(struct_format))
                if not struct_data:
                    break
                trace_data = struct.unpack(struct_format, struct_data)
                timestamp, tid, time_type, task_type, task_id = trace_data
                df_data.append([timestamp/1e3, str(tid), time_type, task_type, task_id])

    df = pd.DataFrame(df_data, columns=df_columns)
    return df

def calculate_elapsed_time(df, task_id):
    TASK_ID = task_id

    task_data = df[df['Task_ID'] == TASK_ID]
    task_data = task_data.sort_values(by=['Timestamp'])
    task_times = []
    begin_timestamp = None
    for _, row in task_data.iterrows():
        if row['Time_Type'] == BEGIN:
            begin_timestamp = row['Timestamp']
        elif row['Time_Type'] == END and begin_timestamp is not None:
            end_timestamp = row['Timestamp']
            time_elapsed = end_timestamp - begin_timestamp
            task_times.append((begin_timestamp, end_timestamp, time_elapsed))
            begin_timestamp = None

    res = pd.DataFrame(task_times, columns=['Begin_Timestamp', 'End_Timestamp', 'Time_Elapsed'])
    return res


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python parser.py <trace_file_prefix>")
        sys.exit(1)
    
    trace_file_prefix = sys.argv[1]
    
    df = decode_data(sys.argv[1])
    df.to_csv("trace_data.csv", index=False)
    derivative_df = calculate_elapsed_time(df, TASK_COMPUTE_DERIVATIVES)
    loop_df = calculate_elapsed_time(df, TASK_LOOP)
    derivative_df.to_csv('derivative.csv', index=False)
    loop_df.to_csv('loop.csv', index=False)
