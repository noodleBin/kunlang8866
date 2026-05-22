
# How to do performance profiling

The purpose of profiling a module is to use tools (here we use google-perftools) to examine the performance problems of a module.

The Century development docker has all the profiling tools you need configured. Therefore, you can do all the following steps in the Century development docker.


## Build Century in profiling mode

First, build Century in profiling mode
```
bash century.sh clean
bash century.sh build_prof
```

## Play a rosbag
To profile a module, you need to provide its input data to make sure the majority of its code can be exercised.
You can start play an information-rich rosbag by
```
rosbag play -l your_rosbag.bag
```

or after Century 3.5, run
```
cyber_record play -f your_record.record
```


## Start module in profiling mode
Start your module with the following command

```
CPUPROFILE=/tmp/${MODULE}.prof /path/to/module/bin/${MODULE} --flagfile=modules/${MODULE}/conf/${MODULE}.conf \
 --${MODULE}_test_mode \
 --${MODULE}_test_duration=60.0 \
 --log_dir=/century/data/log

```
Where `MODULE` is the name of the module you want to test.

or after Century 3.5, use

```
CPUPROFILE=/tmp/${MODULE}.prof mainboard -d /century/modules/${MODULE}/dag/${MODULE}.dag  --flagfile=modules/${MODULE}/conf/${MODULE}.conf \
 --${MODULE}_test_mode \
 --${MODULE}_test_duration=60.0 \
 --log_dir=/century/data/log

```


## The profiling mode gflags
Each module should have a pre-defined `${MODULE}_test_mode`
and `${MODULE}_test_duration` gflag.
These two flags tells the module to run for `${MODULE}_test_duration` amount of time when `${MODULE}_test_mode` is true.

Most of Century modules already have these two gflags. If they does not exist in the module you are interested in, you can define it by yourself. You can refer to gflag `planning_test_mode` and `planning_test_duration` to see how they are being used.

## Create pdf report
Finally you can create a pdf report to view the profiling result.

```
google-pprof --pdf --lines /path/to/module/bin/${MODULE} /tmp/${MODULE}.prof > ${MODULE}_profiling.pdf
```

or after 3.5, run

```
google-pprof --pdf --lines /path/to/module/component_lib/$lib{MODULE}_component_lib.so /tmp/${MODULE}.prof > ${MODULE}_profiling.pdf
```


## Example
Here is an example command of starting the planning module.
```
CPUPROFILE=/tmp/planning.prof /century/bazel-bin/modules/planning/planning \
 --flagfile=modules/planning/conf/planning.conf \
 --log_dir=/century/data/log \
 --planning_test_mode \
 --test_duration=65.0

google-pprof --pdf --lines /century/bazel-bin/modules/planning/planning /tmp/planning.prof > planning_prof.pdf
```

or after Century 3.5, run
```
CPUPROFILE=/tmp/planning.prof mainboard -d /century/modules/planning/dag/planning.dag \
 --flagfile=modules/planning/conf/planning.conf \
 --log_dir=/century/data/log \
 --planning_test_mode \
 --test_duration=65.0

google-pprof --pdf --lines /century/bazel-bin/modules/planning/libplanning_component_lib.so  /tmp/planning.prof > planning_prof.pdf
```
