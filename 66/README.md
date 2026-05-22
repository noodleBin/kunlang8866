# **`Century`无人驾驶项目快速指南**

欢迎来到`Century`无人驾驶项目快速指南，`Century`项目现支持在`Orin`域控制器、`X86`域控制器和普通`X86`系统（包括`X86`架构的台式计算机、笔记本计算机）上部署运行。本文档将介绍`Century`项目的目录结构、部署方法和启动方法。

## 1.  `Century`项目目录结构

**1.1 `Orin`域控制器上的目录结构**

在`Orin`平台上，系统会默认将外接硬盘进行挂载，挂载目录为`/century`，在挂载目录下存放了`Century`项目的所有代码，包括代码、文档、动态库和工具，如下图1-1所示：

![](docs/image/orin_main.png)

<center>图1-1</center>

其中，`/century`下的`hd_map`目录存放高精地图数据，如下图1-2所示：

![](docs/image/hd_map.png)

<center>图1-2</center>

`vehicle_config`目录存放与车辆相关的参数文件，例如传感器标定文件等，如下图1-3所示：

![image-20230102114016671](docs/image/vehicle_cfg.png)

<center>图1-3</center>

`.cache`文件夹存放编译过程中生成的目标文件，在`.cache`文件中有4个子文件夹，其中`repos`存放编译过程中需要下载的部分内容，`bazel`和`build`存放编译生成的目标文件以及相应的依赖库，如下图1-4所示：

![image-20230102114056471](docs/image/bazle.png)

<center>图1-4</center>

如果重新部署代码，可适当清除`bazel`和`build`文件夹下的相关内容，保留其他内容，可加快编译速度。

另外，在用户目录下，如果发现无关文件占据了大量的存储空间，可以自行删除，例如`.cache`，`.log`，`.tmp`等目录都可以删除。但是系统库文件和下述文件不要删除，即**文件夹`~/century`不要删除**：

![image-20230102114349854](docs/image/orin_home.png)

<center>图1-5</center>

该文件存放着是`canbus`模块，不能够删除。

**1.2 `X86`域控制器上的目录结构**

在`X86`域控制器上，代码也放在`/century`路径下，与`Orin`域控制器路径一致，但是`Orin`域控制器是挂接的外部磁盘，`X86`上面是使用本机磁盘，在创建`/century`目录之后需要把该文件夹的权限降低，通过下述命令进行：

```bash
cd /
sudo chmod -R 777 century
```

**1.3 `X86`系统`Docker`目录结构**

`X86`系统指`X86`架构的台式计算机或笔记本计算机。在台式或笔记本计算机上，**要求安装`X86`架构的`Ubuntu 20.04`操作系统**。为了快速验证算法，避免依赖的各类动态库和配置文件带来的各种干扰，在`X86`系统中一般使用`Docker`环境。启动并进入`Docker`，命令如下：

```bash
# 启动Docker
bash docker/scripts/dev_start.sh
# 进入Docker
bash docker/scripts/dev_into.sh
```

成功进入`Docker`后，会默认虚拟出一个用户空间主目录，该目录为`/century`，如图1-6所示：

![](docs/image/docker_main.png)

<center>图1-6</center>

`Docker`环境下的用户空间主目录`/century`对应的实际路径就是宿主PC机上`Century`项目的主目录，例如：`~/code/century`。因此在`Docker`环境下`/century`的内容就跟宿主PC机上`~/code/century`目录下的内容一致。

## 2. 代码部署方法

在`X86`域控制器上，启动VPN，输入用用户名和密码，完成登录。按照上一节的内容，进入`Century`项目目录并拉取最新的代码，并进行增量编译，具体方法描述如下。

### 2.1. 在`X86`域控制器上更新代码并构建工程

在`X86`域控制器上进入工作目录，并拉取最新代码：

```bash
cd ~/century
git pull --rebase upstream master
```
代码拉取完成之后，开始构建：

```bash
# 构建工程
bash century.sh build
```

如果采用增量编译方式，则需要较少时间（约3~5分钟）；如果全量重新编译，则必须删除当前目录下的`.cache`目录，然后再执行上述编译命令，则需要花费大概1小时。

### 2.2. 在`Orin`域控制器上更新代码并构建工程

由于`Orin`域控制器上无法安装VPN，因此必须通过手工拷贝的方式更新代码，从其他平台拉取最新的代码，并打包：

```bash
tar -cvf century.tar.gz century
```

打包操作不能省略，因为项目中有多个软连接，在copy过程中会失效，因此copy之前必须先进行打包。

打包完成之后，把代码手工拷贝到`Orin`域控制器挂载的目录`/century`，如图1-1所示；然后把tar包解压缩，为了保证只有一级`century`目录，**在解压的时候，一定要添加`strip_components`参数**:

```bash
tar -xvf century.tar.gz --strip-components=1
```

解压缩完成之后，进行编译.

```bash
bash century.sh build
```

### 2.3 在`X86`系统`Docker`环境下更新代码并构建工程

在宿主PC机上进入工作目录，拉取最新代码，并启动docker环境编译

```bash
# 进入工作目录
cd ~/century
# 更新代码
git pull --rebase upstream master
# 启动Docker
bash docker/scripts/dev_start.sh
# 进入Docker
bash docker/scripts/dev_into.sh
# 构建工程
bash century.sh build
# 如果需要将本机代码提交到服务器，请先执行如下命令，确认所有检查顺利通过
bash century.sh check
```

## 3. 启动`Century`无人驾驶系统

在`Orin`或`X86`域控制器上，进入`Century`项目目录，运行脚本：

```bash
bash launch_century_system.sh
```
脚本会根据不同的平台执行各自需要执行的任务，一般而言，会在`X86`域控制器上运行`socket_can_rx`和`socket_can_tx`。如果需要在`Orin`域控制器上运行以上任务，则必须手动添加相应的脚本。

在`X86`域控制器上还可以通过`Dreamview`启动对应的模块，步骤如下：

```bash
cd ~/century
bash century.sh build
bash scripts/bootstrap.sh
```

`Dreamview`启动后，通过界面可启动`control`、`planning`、`routing`等模块，如图3-1所示：

![dreamview](./docs/image/dreamview.png)
<center>图3-1</center>

另外，可通过`cyber_monitor`查看当前启动模块发出的消息，如图3-2所示：

![cyber_monitor](./docs/image/cyber_monitor.png)
<center>图3-2</center>