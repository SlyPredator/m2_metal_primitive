#!/bin/bash
set -e
cd "$(dirname "$0")/.."

WEB=0; [ "$1" = "--web" ] && WEB=1

if [ "$WEB" = "1" ]; then
  command -v emcc >/dev/null || { echo "emcc not found — install emscripten"; exit 1; }
  RLW=vendor/raylib-5.5_webassembly
  mkdir -p build/web
  cp -rf web/assets build/web/
  emcc -O3 web/m2_demo.c -o build/web/index.html \
    -I web -I "$RLW/include" \
    "$RLW/lib/libraylib.a" \
    -sUSE_GLFW=3 -sUSE_WEBGL2=1 -sASYNCIFY -sFORCE_FILESYSTEM=1 \
    -sINITIAL_MEMORY=32MB -sALLOW_MEMORY_GROWTH \
    -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES3 \
    --preload-file web/m2_meshes.bin@web/m2_meshes.bin \
    --shell-file web/shell.html
  echo "Built: build/web/index.html"
else
  RL=vendor/raylib-5.5_linux_amd64
  mkdir -p build
  gcc -O2 web/m2_demo.c -o build/m2demo \
    -I web -I "$RL/include" \
    "$RL/lib/libraylib.a" -lm -lGL -lpthread -ldl -lrt -lX11
  echo "Built: build/m2demo"
fi
