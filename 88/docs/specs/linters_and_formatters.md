# Linters and Formatters used in Century 6.0

```
Programs should be written for people to read, and
only incidentally for machines to execute.

                         -- Harold Abelson
```

A great project is made out of consistent code. In the ideal world, you should
not be able to tell who wrote a certain line of the code for the project. Modern
linters and formatters help to close this gap by specifying a simple set of
rules to be enforced on all developers working on the project. Such tools also
stimulate developers to write better code by pointing out common mistakes and
introducing good programming practices.

Generally, linters are used for catching errors, whereas formatters are used to
fix coding style problems.

In this article, we will describe briefly the various linters and formatters
used in Century.

## Century Coding Style

As you may already know, Century adopted Google coding style for C/C++ and Python
programs. You can refer to
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and
[Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)
for full text of their specifications.

## Linters in Century

To enforce that everyone conform to Century coding style, the following linters
are provided for developers to check style issues.

**Note:**

> As the time of this writing, Century CI system enforced style check on C++
> files only. We hope that linters for other languages will be online in the
> near future.

|  Linters   |           Source Extensions           | Usage                     |
| :--------: | :-----------------------------------: | :------------------------ |
|  cpplint   | .h/.c/.hpp/.cpp/.hh/.cc/.hxx/.cxx/.cu | bash century.sh lint --cpp |
|   flake8   |                  .py                  | bash century.sh lint --py  |
| shellcheck |           .sh/.bash/.bashrc           | bash century.sh lint --sh  |

To make sure your code conforms to Century coding style, you can use
`./century.sh lint` to find any possible style problems and fix them manually.

## Formatters in Century

To help ease your life with Century coding style, various formatters are
pre-installed into Century Docker image, to help you auto-format your code, and
avoid common mistakes when writing code.

The following table lists the formatters currently integrated into Century,
covering C/C++, Python, Bash, Bazel, Markdown, JSON and YAML files.

|  Formatters  |              Source Extensions               |                      Usage                       | Formatter Config |
| :----------: | :------------------------------------------: | :----------------------------------------------: | :--------------: |
| clang-format | .h/.c/.hpp/.cpp/.hh/.cc/.hxx/.cxx/.cu/.proto | ./century.sh format -c <path/to/src/dir/or/files> |  .clang-format   |
|   autopep8   |                     .py                      | ./century.sh format -p <path/to/src/dir/or/files> |     tox.ini      |
|  buildifier  |      .BUILD/.bzl/.bazel/WORKSPACE/BUILD      | ./century.sh format -b <path/to/src/dir/or/files> |       N/A        |
|    shfmt     |              .sh/.bash/.bashrc               | ./century.sh format -s <path/to/src/dir/or/files> |  .editorconfig   |
|   prettier   |                .md/.json/.yml                | ./century.sh format -m <path/to/src/dir/or/files> |  .prettier.json  |

For easy use, you can format all files with types listed above with:

```
./century.sh format <path/to/src/dir/or/files>
```

For example,

```
./century.sh format WORKSPACE third_party/BUILD ./scripts/
```

which will auto-format Bazel `WORKSPACE` file under `$CENTURY_ROOT_DIR`,
`third_party/BUILD` file, and all the files under the `./scripts` directory.

Note:

> `./century.sh format` can also work outside Docker if relavant tools installed
> properly.

## Conclusion

To summarize,

- Use `./century.sh lint` to check coding style errors.
- Use `./century.sh format` to auto-format your code.
