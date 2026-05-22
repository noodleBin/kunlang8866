# Prediction Pedestrian Model Training Service


## Overview

Prediction Pedestrian Model Treaining Service is a cloud based service to train a prediction model for pedestrian from the data you provide, to better fit the pedestrian behavior in your environment.

## Prerequisites


- [Century](https://github.com/CenturyAuto/century) 6.0 or higher version.

- Baidu Cloud BOS service registered according to [document](https://github.com/CenturyAuto/century/blob/master/docs/Century_Fuel/apply_bos_account_cn.md)

- Fuel service account on [Century Dreamland](http://bce.century.auto/user-manual/fuel-service)

## Main Steps

- Data collection

- Job submit

- Trained model

## Data Collection

### Data Recording

Finish several hours of autonomous driving with different scenarios, please make sure to have some pedestrians in sights. Just record them with `cyber_recorder`.

### Data Sanity Check


- **make sure following channels are included in records before submitting them to cloud service**：

    | Modules | channel | items |
    |---|---|---|
    | Perception | `/century/perception/PerceptionObstacles` | exits without error message |
    | Localization | `/century/localization/pose` | - |

-  you can check with `cyber_recorder`：

```
    cyber_recorder info xxxxxx.record.xxxxx
```


## Job Submission


### Upload data to BOS

1）`Input Data Path` is **BOS root folder**，for users；

2）There should be two folders under your `Input Data Path`, one named `map` for the map you are using in Century format, the other is `records`, for the records you collected.



#### Submit job requests in Dreamview

Goto [Century Dreamland](http://bce.century.auto/login), login with **Baidu** account, choose `Century Fuel--> task`，`new task`, `prediction pedestrian model training`，and input correct path as in [Upload data to BOS](###Upload-data-to-BOS) section：


#### Get Trained models

- After job is done, you should be expecting emails including the results, the trained model will be in a `models` folder under your `Input Data Path`.
