import numpy as np
def read_pcd_with_intensity(pcd_path):
    # read header
    with open(pcd_path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('utf-8').strip()
            header.append(line)
            if line.startswith('DATA'):
                break

    # decode param type and size
    fields, size, type_ = None, None, None
    for line in header:
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
        elif line.startswith('SIZE'):
            size = list(map(int, line.split()[1:]))
        elif line.startswith('TYPE'):
            type_ = line.split()[1:]

    if fields is None or size is None or type_ is None:
        raise ValueError("Invalid PCD header: missing FIELDS/SIZE/TYPE")

    if not len(fields) == len(size) == len(type_):
        raise ValueError("FIELDS/SIZE/TYPE length mismatch")

    # construct dtype with TYPE and SIZE
    def get_numpy_dtype(t, s):
        if t == 'F':
            if s == 4:
                return np.float32
            elif s == 8:
                return np.float64
        elif t == 'U':
            if s == 1:
                return np.uint8
            elif s == 2:
                return np.uint16
            elif s == 4:
                return np.uint32
        elif t == 'I':
            if s == 1:
                return np.int8
            elif s == 2:
                return np.int16
            elif s == 4:
                return np.int32
        raise ValueError(f"Unsupported TYPE/SIZE combination: TYPE={t}, SIZE={s}")

    dtype = np.dtype([(f, get_numpy_dtype(t, s)) for f, t, s in zip(fields, type_, size)])
    print(dtype)

    # cal start positon
    data_offset = len('\n'.join(header)) + 1
    print(data_offset)
    data = np.fromfile(pcd_path, dtype=dtype, offset=data_offset)

    # check fields
    required = {'x', 'y', 'z', 'intensity', 'ring'}
    if not required.issubset(data.dtype.names):
        raise ValueError(f"Missing required fields. Expected at least: {required}")

    # construct output data (automatically determine whether timestamp_2us exists)
    base_fields = ['x', 'y', 'z', 'intensity', 'ring']
    print(data['x'])
    arrs = [data[f].astype(np.float32) for f in base_fields]

    if 'timestamp' in data.dtype.names:
        arrs.append(data['timestamp'].astype(np.float32))

    all_data = np.vstack(arrs).T

    # filter out points with NaN
    valid_mask = ~np.isnan(all_data).any(axis=1)
    return all_data[valid_mask]

point = read_pcd_with_intensity("data/0704_cone_out/202507040930_record/lidar/bp_front_left/1751592601.131494868.pcd")