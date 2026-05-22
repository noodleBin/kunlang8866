# Navigator

## Introduction
The Navigator converts the smoothed navigation data collected by a vehicle into a `NavigationInfo` format and sends it to the real-time relative map module with the message of `FLAGS_navigation_topic` for dynamically generating relative maps.


## Usage
Example 1:
```bash
bash scripts/navigator.sh /century/data/bag/path_csc_middle.bag.txt.smoothed /century/data/bag/path_csc_left.bag.txt.smoothed  /century/data/bag/path_csc_right.bag.txt.smoothed
```
or
```bash
cd /century/data/bag
bash /century/scripts/navigator.sh path_csc_middle.bag.txt.smoothed path_csc_left.bag.txt.smoothed  path_csc_right.bag.txt.smoothed
```

Example 2:
```bash
bash scripts/navigator.sh data/bag/csc/path_csc_left.bag.txt.smoothed  data/bag/csc/path_csc_right.bag.txt.smoothed
```
or
```bash
cd /century/data/bag/csc
bash /century/scripts/navigator.sh path_csc_left.bag.txt.smoothed  path_csc_right.bag.txt.smoothed
```

## Other Instructions
You can change the configuration information by modifying the `/century/modules/map/relative_map/conf/navigator_config.pb.txt` file.