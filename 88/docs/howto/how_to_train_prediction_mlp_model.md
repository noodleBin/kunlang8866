## How to train the MLP Deep Learning Model

### Prerequisites
There are 2 prerequisites to training the MLP Deep Learning Model:
#### Download and Install Anaconda
* Please download and install Anaconda from its [website](https://www.anaconda.com/download)

#### Install Dependencies
Run the following commands to install the necessary dependencies:
* **Install numpy**: `conda install numpy`
* **Install tensorflow**: `conda install tensorflow`
* **Install keras**: `conda install -c conda-forge keras`
* **Install h5py**: `conda install h5py`
* **Install protobuf**: `conda install -c conda-forge protobuf`
* **Install PyTorch**: `conda install -c pytorch pytorch`

### Train the Model
The following steps are to be followed in order to train the MLP model using the released demo data. For convenience, we denote `CENTURY` as the path of the local century repository, for example, `/home/username/century`

1. Create a folder to store offline prediction data using the command `mkdir CENTURY/data/prediction` if it does not exist

1. Start dev docker using `bash docker/scripts/dev_start.sh` under the century folder

1. Enter dev docker using `bash docker/scripts/dev_into.sh` under century folder

1. In docker, under `/century/`, run `bash century.sh build` to compile

1. In docker, under `/century/`, copy the demo record into `/century/data/prediction` by the command: `cp /century/docs/demo_guide/demo_3.5.record /century/data/prediction/`

1. In docker, under `/century/`, run the bash script for feature extraction: `bash modules/tools/prediction/mlp_train/feature_extraction.sh /century/data/prediction/ century/data/prediction/`, then the feature files will be generated in the folder `/century/data/prediction/`.

1. Exit docker, train the cruise model and junction model according to `CENTURY/modules/tools/prediction/mlp_train/cruiseMLP_train.py` and `CENTURY/modules/tools/prediction/mlp_train/junctionMLP_train.py`
