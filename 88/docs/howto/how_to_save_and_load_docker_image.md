# How to Save and Load Century Docker Images

Considering that Century Docker images are usually serveral Gigabytes in size,
you can pull them over WIFI or other stable Internet connections, and then
copy the relavant images to vehicle.

## Save Docker Images

After you have pulled the relavant Docker image, you can save it as a tarball
by running the following command:

```bash
# docker save -o <path/to/saved/image/tar> <repo:tag>
docker save -o century_dev.tar centuryauto/century:dev-x86_64-18.04-20200823_0534
```

## Load the Docker Image

With the taballs copied to vehicle, you can reload the relavant Century
Docker image by running:

```bash
# docker load -i <path/to/docker/tarball>
docker load -i century_dev.tar
```

