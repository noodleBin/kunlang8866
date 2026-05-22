# Steps for

- Generating navigation data from bag and
- Manually sending the data to /century/navigation topic

### Step 1: In dev docker, extract path data from bags

```
dev_docker:/century$cd /modules/tools/navigator
dev_docker:/century/modules/tools/navigator$python extractor.py path-to-bags/*.bag
```

A path file will be generated in

```
dev_docker:/century/modules/tools/navigator$
```

With format of

```
path_[first_bag_name].bag.txt
```



### Step2: (Optional) Verify the extracted path is correct

dev_docker:/century/modules/tools/navigator$python viewer_raw.py path_[bag_name].bag.txt

### Step3: Smooth the path

```
dev_docker:/century/modules/tools/navigator$./smooth.sh /century/modules/tools/navigator/path_[first_bag_name].bag.txt 200
```

200 is the parameter for smooth length. If the smooth is failed, try to change this parameter to make the smooth pass. The preferred number is between 150 and 200.

A smoothed data file, path_[first_bag_name].bag.txt.smoothed, is generated under folder

```
dev_docker:/century/modules/tools/navigator$
```

### Step4: (Optional) Verify the smoothed data

```
dev_docker:/century/modules/tools/navigator$ python viewer_smooth.py path[first_bag_name].bag.txt path[first_bag_name].bag.txt.smoothed
```



### Step5: Send /century/navigation topic

Run follow command to send /century/navigation  data

```
dev_docker:/century/modules/tools/navigator$python navigator.py path_[first_bag_name].bag.txt.smoothed
```