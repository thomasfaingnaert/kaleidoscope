# Kaleidoscope tutorial

Kaleidscope tutorial (https://llvm.org/docs/tutorial/MyFirstLanguageFrontend/index.html).
Tested with LLVM 11.0.0.

## Compilation

```bash
mkdir build && cd build
cmake -DLLVM_DIR=/path/to/llvm/lib/cmake/llvm -GNinja ..
ninja
```
