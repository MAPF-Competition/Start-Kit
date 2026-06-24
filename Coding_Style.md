# Coding Style

The start kit uses C++17. Prefer clear, explicit types in public and starter code so submissions remain easy to read and debug.

## Linting

Run the style check from a configured build directory:

```shell
cmake --build build --target lint
```

The `lint` target rejects use of the C++ `auto` keyword in start-kit source and test files. Use explicit types instead.
