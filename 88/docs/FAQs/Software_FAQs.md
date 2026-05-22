# Software FAQs

## Can other operating systems besides Ubuntu be used?

We have only tested on Ubuntu which means it's the only operating system we
currently officially support. Users are always welcome to try different
operating systems and can share their patches with the community if they are
successfully able to use them.

---

## How can I perform step-by-step debugging?

The majority of bugs can be found through logging (using AERROR, AINFO, ADEBUG).
If step-by-step debugging is needed, we recommend using gdb.

---

## How do I add a new module

Century currently functions as a single system, therefore before adding a module
to it, understand that there would be a lot of additional work to be done to
ensure that the module functions perfectly with the other modules of Century.
Simply add your module to the `modules/` folder. You can
use `modules/routing` as an example, which is a relatively simple module. Write
the BUILD files properly and century.sh will build your module automatically

---

## Build error "docker: Error response from daemon: failed to copy files: userspace copy failed":

An error message like this means that your system does not have enough space to
build Century. To resolve this issue, run the
following command to free up some space on your host:

```
docker/setup_host/cleanup_resources.sh
```

Run `bash century.sh clean -a` inside Docker to free more.

---

## Bootstrap error: unix:///tmp/supervisor.sock refused connection

There could be a number of reasons why this error occurs. Please follow the
steps recommended in the
[following thread](https://github.com/CenturyAuto/century/issues/5344). There are
quite a few suggestions. If it still does not work for you, comment on the
thread mentioned above.

---

## My OS keeps freezing when building Century 3.5?

If you see an error like this, you do not have enough memory to build Century.
Please ensure that you have at least **16GB** memory available before building
Century. You could also find `--jobs=$(nproc)` in century.sh file and replace it
with `--jobs=2`. This will make build process to use only 2 cores. Building will
be longer, but will use less memory.

---

**More Software FAQs to follow.**
