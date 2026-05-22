# Bazel in Century

## Overview

Century uses [Bazel](https://bazel.build) as its underlying build system. Bazel
is an open-source build and test tool with a human-readable, high-level build
language suitable for medium and large projects. You may notice the `WORKSPACE`
file under the root directory, and many `BUILD`, `*.BUILD`, `workspace.bzl`
files in sub-directories (e.g. `third_party`). They are all Bazel files.

## Bazel Settings in Century

### bazelrc

The content of `.bazelrc` (Century's overall Bazel configuration) under the root
directory is as follows as of this writing:

```
try-import %workspace%/tools/bazel.rc
try-import %workspace%/.century.bazelrc
```

`tools/bazel.rc` is for general settings, while `.century.bazelrc` was generated
with the `config` sub-command of `century.sh`:

You can run

```bash
./century.sh config --interactive
```

to configure it interactively, or

```bash
./century.sh config --noninteractive
```

should be run to configure it non-interactively.

### .bazelignore

Besides `.bazelrc`, the `.bazelignore` file under Century workspace also governs
Bazel’s behavior. Similar to `.gitignore`, `.bazelignore` is used to specify
directories for Bazel to ignore. Currently, the `scripts`, `docker`, `docs`
directories were specified.

### Bazel Distribution Files Directory

Within `.century.bazelrc`, two directories were specified for Bazel:

```
startup --output_user_root="/century/.cache/bazel"
common --distdir="/century/.cache/distdir"
```

The startup option `--output_user_root` was used to specify Bazel output
directories (Ref:
[Bazel Docs: Output Directory Layout](https://docs.bazel.build/versions/master/output_directories.html#output-directory-layout)).
We specify it within Century root directory so that it can be mounted into Docker
container by `docker/scripts/dev_start.sh` together with Century root directory
on the host.

According to
[Bazel Docs: Distribution Files Directories](https://docs.bazel.build/versions/master/guide.html#distribution-files-directories),

> Using the `--distdir=/path/to/directory` option, you can specify additional
> read-only directories to look for files instead of fetching them. A file is
> taken from such a directory if the file name is equal to the base name of the
> URL and additionally the hash of the file is equal to the one specified in the
> download request.

Since this option is especially useful for users with not-stable-enough network
connection, Century enabled a specific environment variable for that:
`CENTURY_BAZEL_DIST_DIR`. You can configure it in `cyber/setup.bash`, and then
run the following command for it to take effect.

```
source cyber/setup.bash
./century.sh config --noninteractive
```

## Bazel Build, Test and Coverage

Please refer to
[Century Build and Test Explained](century_build_and_test_explained.md).

### Century's Special

### Proto Files

In Century 6.0, we recommend that proto files for each module be placed under
some separate directory, say, `modules/a/proto`.

Then you can run the following command to generate C++ and Python targets for
all the proto files under that directory:

```
scripts/proto_build_generator.py modules/a/proto/BUILD
```

### [Breaking Change] Python Files

Starting from Century 6.0, Python libraries/binaries were also managed by Bazel.

What you do in previous Century releases, say,

```
python3 modules/tools/mapshow/mapshow.py
```

, now should be done in either of the following approaches:

```
./bazel-bin/modules/tools/mapshow/mapshow # Approach #1
bazel run modules/tools/mapshow:mapshow   # Approach #2
```

## Further Reading

Please refer to [Bazel Docs](https://docs.bazel.build) for more on Bazel.

Thanks for reading!
